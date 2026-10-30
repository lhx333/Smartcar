"""E35 transparent UART telemetry and control console."""
from __future__ import annotations
import queue, threading, time, tkinter as tk
from datetime import datetime
from protocol import Decoder, Recorder
from tkinter import messagebox, ttk
import serial
from serial.tools import list_ports

class E35App:
    def __init__(self, root):
        self.root=root; root.title("E35 无线串口透传上位机"); root.geometry("1080x720"); root.minsize(850,580)
        self.port=None; self.lock=threading.Lock(); self.events=queue.Queue(); self.tx=queue.Queue(); self.buf=""; self.count=0; self.imu_mode="未收到模式声明"; self.latest_state="未知"
        self.port_var=tk.StringVar(); self.baud=tk.StringVar(value="115200"); self.status=tk.StringVar(value="未连接"); self.summary=tk.StringVar(value="等待协议 v1 二进制遥测数据…")
        self.command_status=tk.StringVar(value="命令确认：等待上位机命令")
        self.pwm_percent=tk.StringVar(value="25"); self.pwm_hold_s=tk.StringVar(value="5")
        self.circle_yaw=tk.StringVar(value="45"); self.circle_servo=tk.StringVar(value="8"); self.circle_v1=tk.StringVar(value="0.4"); self.circle_v2=tk.StringVar(value="0.6")
        self.start_mode=tk.StringVar(value="模块1：按PWM/时间")
        self.pending_config=None
        self.config_generation=0
        self.calibration=tk.StringVar(value="IMU 标定：连接串口后可启动")
        self.debug_status=tk.StringVar(value="按钮遥控：未启动"); self.debug_active=False
        self.manual_active=False; self.manual_focus_lost=False
        self.manual_keys={"up":False,"down":False,"left":False,"right":False}
        self.manual_sources={key:{"keyboard":False,"mouse":False} for key in self.manual_keys}
        self.joy_controls=[]
        self.hex_view=tk.BooleanVar(); self.hex_send=tk.BooleanVar(); self.crlf=tk.BooleanVar(value=True)
        self.log_tx=queue.Queue(); self.reader_thread=None; self.pending_reading=None; self.last_valid=0; self.last_draw=0; self.bad_frames=0; self.missing_frames=0; self.log_path=""
        self._ui(); self.refresh_ports(); threading.Thread(target=self._writer,daemon=True).start(); root.after(20,self._poll); root.after(50,self._manual_heartbeat); root.protocol("WM_DELETE_WINDOW",self.close_app)
        root.bind_all("<KeyPress>",self._on_key_press,add="+"); root.bind_all("<KeyRelease>",self._on_key_release,add="+")
    def _ui(self):
        style=ttk.Style(); style.configure("Title.TLabel",font=("Microsoft YaHei UI",18,"bold")); style.configure("Treeview",rowheight=27)
        outer=ttk.Frame(self.root,padding=16); outer.pack(fill="both",expand=True)
        ttk.Label(outer,text="E35 无线串口透传上位机",style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer,text="电脑端 E35 通过 USB 转串口接入；START/STOP 与传感器数据经 E35 透明传输。",foreground="#667085").pack(anchor="w",pady=(4,12))
        conn=ttk.LabelFrame(outer,text="串口连接",padding=10); conn.pack(fill="x")
        row=ttk.Frame(conn); row.pack(fill="x")
        ttk.Label(row,text="COM口").pack(side="left"); self.ports=ttk.Combobox(row,textvariable=self.port_var,width=32,state="readonly"); self.ports.pack(side="left",padx=6)
        ttk.Button(row,text="刷新",command=self.refresh_ports).pack(side="left")
        ttk.Label(row,text="波特率").pack(side="left",padx=(18,5)); self.baudbox=ttk.Combobox(row,textvariable=self.baud,width=12,state="readonly",values=("9600","19200","38400","57600","115200","230400","460800")); self.baudbox.pack(side="left")
        self.conn_btn=ttk.Button(row,text="连接串口",command=self.toggle); self.conn_btn.pack(side="left",padx=10)
        ttk.Label(row,textvariable=self.status,foreground="#246bfd").pack(side="left",padx=6)
        actions=ttk.Frame(conn); actions.pack(fill="x",pady=(10,0))
        self.start=ttk.Button(actions,text="发车 START",command=lambda:self.control("START"),state="disabled"); self.start.pack(side="left")
        self.stop=ttk.Button(actions,text="停车 STOP",command=lambda:self.control("STOP"),state="disabled"); self.stop.pack(side="left",padx=8)
        self.debug=ttk.Button(actions,text="手动遥控 DEBUG",command=lambda:self.control("DEBUG"),state="disabled"); self.debug.pack(side="left",padx=(0,8))
        self.calibrate=ttk.Button(actions,text="IMU 零偏标定",command=self.start_calibration,state="disabled"); self.calibrate.pack(side="left",padx=(0,8))
        ttk.Label(actions,textvariable=self.calibration,foreground="#344054").pack(side="left",padx=4)
        pwmrow=ttk.Frame(conn); pwmrow.pack(fill="x",pady=(7,0))
        ttk.Label(pwmrow,text="发车模式").pack(side="left")
        ttk.Combobox(pwmrow,textvariable=self.start_mode,state="readonly",width=24,
                     values=("模块1：按PWM/时间","模块4：按航向角/速度","其他模块：普通START")).pack(side="left",padx=(5,12))
        ttk.Label(pwmrow,text="模块1开环 PWM (%)").pack(side="left")
        ttk.Spinbox(pwmrow,from_=10,to=40,increment=1,textvariable=self.pwm_percent,width=5).pack(side="left",padx=(5,12))
        ttk.Label(pwmrow,text="保持时间 (s)").pack(side="left")
        ttk.Spinbox(pwmrow,from_=2,to=5,increment=1,textvariable=self.pwm_hold_s,width=4).pack(side="left",padx=5)
        ttk.Label(pwmrow,text="到时 PWM 归零；舵机依据 IMU 和编码器修正直线",foreground="#667085").pack(side="left",padx=12)
        circle_row=ttk.Frame(conn);circle_row.pack(fill="x",pady=(5,0))
        ttk.Label(circle_row,text="模块4目标航向变化角 (°)").pack(side="left")
        ttk.Spinbox(circle_row,from_=-90,to=90,increment=1,textvariable=self.circle_yaw,width=5).pack(side="left",padx=(5,12))
        ttk.Label(circle_row,text="舵机偏角幅值 (°)").pack(side="left")
        ttk.Spinbox(circle_row,from_=2,to=15,increment=1,textvariable=self.circle_servo,width=4).pack(side="left",padx=(5,12))
        ttk.Label(circle_row,text="转弯速度 (m/s)").pack(side="left")
        ttk.Spinbox(circle_row,from_=0.2,to=1.0,increment=0.1,textvariable=self.circle_v1,width=5).pack(side="left",padx=(5,12))
        ttk.Label(circle_row,text="回正后直线速度 (m/s)").pack(side="left")
        ttk.Spinbox(circle_row,from_=0.2,to=1.0,increment=0.1,textvariable=self.circle_v2,width=5).pack(side="left",padx=5)
        ttk.Label(conn,textvariable=self.debug_status,foreground="#246bfd").pack(anchor="w",pady=(8,0))
        ttk.Label(conn,textvariable=self.command_status,foreground="#18794e").pack(anchor="w",pady=(4,0))
        pad=ttk.LabelFrame(conn,text="手动操纵（按住按钮或键盘）",padding=6); pad.pack(anchor="w",pady=(8,0))
        self._add_hold_button(pad,"↑ 加速","up",0,1)
        self._add_hold_button(pad,"← 左转","left",1,0)
        ttk.Label(pad,text="松开转向按钮回中").grid(row=1,column=1,padx=8,pady=4)
        self._add_hold_button(pad,"右转 →","right",1,2)
        self._add_hold_button(pad,"↓ 减速","down",2,1)
        ttk.Label(conn,text="按住 ↑/W 加速至 2 m/s，按住 ↓/S 减速至 0；松开后保持当前速度目标。按住 ←/A 或 →/D 转向 ±20°，松开后舵机回到 99° 中位。转向与加减速可同时操作；当前输入对应按钮会点亮。窗口失去焦点时自动减速。",foreground="#667085",wraplength=1000).pack(anchor="w",pady=(4,0))
        box=ttk.LabelFrame(outer,text="串口数据与传感器",padding=10); box.pack(fill="both",expand=True,pady=(12,0))
        bar=ttk.Frame(box); bar.pack(fill="x"); ttk.Button(bar,text="清空",command=self.clear).pack(side="right"); ttk.Checkbutton(bar,text="HEX 显示",variable=self.hex_view).pack(side="right",padx=8)
        tabs=ttk.Notebook(box); tabs.pack(fill="both",expand=True,pady=8); logtab=ttk.Frame(tabs,padding=5); sens=ttk.Frame(tabs,padding=7); tabs.add(logtab,text="串口日志"); tabs.add(sens,text="传感器实时读数")
        self.log=tk.Text(logtab,state="disabled",wrap="word",font=("Consolas",10),bg="#101828",fg="#d5e4ff"); self.log.pack(fill="both",expand=True); self.log.tag_configure("tx",foreground="#84e1bc"); self.log.tag_configure("info",foreground="#a9bce0")
        ttk.Label(sens,textvariable=self.summary,wraplength=1000,foreground="#344054").pack(fill="x",anchor="w",pady=(0,8))
        cols=("time_s","tick","state","error","yaw","yaw_error","gz","vl","vr","turn","servo_offset","duty_l","duty_r","x","y","progress","path","speed_target","flags")
        labels=("采样时间 s","拍号","状态","错误码","航向 °","航向误差 °","Z角速度 °/s","左速 m/s","右速 m/s","转向指令","舵机偏移 °","左PWM %","右PWM %","X m","Y m","进度 m","路径段","目标 m/s","标志位")
        self.reading_cols=cols
        self.tree=ttk.Treeview(sens,columns=cols,show="headings");
        for c,l in zip(cols,labels): self.tree.heading(c,text=l); self.tree.column(c,width=105,minwidth=70,anchor="center")
        self.tree.pack(fill="both",expand=True)
        scroll=ttk.Scrollbar(sens,orient="horizontal",command=self.tree.xview); scroll.pack(fill="x"); self.tree.configure(xscrollcommand=scroll.set)
        self.log_status=tk.StringVar(value="连接后自动保存全部采样到 logs：CSV + 事件 JSONL + 原始二进制")
        ttk.Label(sens,textvariable=self.log_status,wraplength=1000).pack(fill="x")
        sendrow=ttk.Frame(box); sendrow.pack(fill="x"); self.entry=ttk.Entry(sendrow); self.entry.pack(side="left",fill="x",expand=True); self.entry.bind("<Return>",lambda _e:self.send_custom())
        ttk.Button(sendrow,text="发送",command=self.send_custom).pack(side="left",padx=6); ttk.Checkbutton(sendrow,text="HEX发送",variable=self.hex_send).pack(side="left"); ttk.Checkbutton(sendrow,text="追加CRLF",variable=self.crlf).pack(side="left",padx=5)
    def _add_hold_button(self,parent,label,key,row,column):
        button=tk.Button(parent,text=label,state="disabled",relief="raised",background="#e6eaf0",activebackground="#c7f0d8",disabledforeground="#98a2b3",padx=12,pady=5)
        button.manual_key=key
        button.grid(row=row,column=column,padx=4,pady=3,sticky="ew")
        button.bind("<ButtonPress-1>",lambda _event,k=key:self._set_manual_button(k,True),add="+")
        button.bind("<ButtonRelease-1>",lambda _event,k=key:self._set_manual_button(k,False),add="+")
        self.joy_controls.append(button)
    def _set_manual_button(self,key,pressed):
        if not self.manual_active:return
        self._set_manual_source(key,"mouse",pressed)
    def _set_manual_source(self,key,source,pressed):
        if not self.manual_active:return
        old=self.manual_keys[key]
        self.manual_sources[key][source]=pressed
        self.manual_keys[key]=any(self.manual_sources[key].values())
        button=next((b for b in self.joy_controls if b.manual_key==key),None)
        if button:
            if self.manual_keys[key]:button.configure(background="#42c978",relief="sunken")
            else:button.configure(background="#e6eaf0",relief="raised")
        if old!=self.manual_keys[key]:self._manual_command()
    def _handle_manual_key(self,event,pressed):
        if not self.manual_active:return None
        mapping={"Up":"up","Down":"down","Left":"left","Right":"right",
                 "w":"up","W":"up","s":"down","S":"down",
                 "a":"left","A":"left","d":"right","D":"right"}
        key=mapping.get(event.keysym)
        if key:
            self._set_manual_source(key,"keyboard",pressed)
            return "break"
        return None
    def _on_key_press(self,event):return self._handle_manual_key(event,True)
    def _on_key_release(self,event):return self._handle_manual_key(event,False)
    def refresh_ports(self):
        found=list(list_ports.comports()); vals=[f"{p.device} — {p.description}" for p in found]; old=self.port_var.get(); self.ports["values"]=vals
        self.port_var.set(old if old in vals else (vals[0] if vals else ""))
        if not vals: self._log("未发现串口。连接电脑端 E35 的 USB 转串口后点击刷新。\n","info")
    def toggle(self):
        if self.port and self.port.is_open: self._disconnect(); return
        name=self.port_var.get().split(" — ",1)[0] if self.port_var.get() else ""
        if not name: messagebox.showinfo("选择串口","请选择电脑端 E35 对应的 COM 口。",parent=self.root); return
        try: p=serial.Serial(name,int(self.baud.get()),bytesize=8,parity="N",stopbits=1,timeout=.1,write_timeout=1)
        except (serial.SerialException,ValueError,OSError) as e: messagebox.showerror("串口打开失败",f"{e}\n检查 COM 口占用及 E35 UART 波特率。",parent=self.root); return
        self.latest_state="未知"; self.debug_active=False; self._deactivate_manual("按钮遥控：已连接；先标定至 ARMED")
        self.debug.configure(state="normal")
        self.last_valid=0; self.pending_reading=None; self.count=0; self.bad_frames=0; self.missing_frames=0
        self.port=p; self.buf=""; self.status.set(f"已打开 {name} · {self.baud.get()} bps，等待数据")
        self.conn_btn.configure(text="断开串口"); self.start.configure(state="disabled"); self.stop.configure(state="normal"); self.calibrate.configure(state="normal"); self.calibration.set("IMU 标定：等待状态/可手动启动"); self.ports.configure(state="disabled"); self.baudbox.configure(state="disabled")
        self._log(f"[打开 {name}，{self.baud.get()} bps]\n","info"); self.reader_thread=threading.Thread(target=self._reader,args=(p,),daemon=True); self.reader_thread.start()
    def _reader(self,p):
        decoder=Decoder(); recorder=None; stalled=False; last=time.monotonic()
        try:
            try:
                recorder=Recorder(); recorder.event("port",{"name":p.port,"baud":p.baudrate})
                self.events.put(("log_path",(p,str(recorder.path))))
            except OSError as e:self.events.put(("log_error",(p,str(e))))
            while p.is_open:
                while True:
                    try:owner,label=self.log_tx.get_nowait()
                    except queue.Empty:break
                    if owner is p and recorder:recorder.event("tx",label)
                n=p.in_waiting; data=p.read(n if n else 1)
                now=time.monotonic()
                if data:
                    frames=decoder.feed(data,now)
                    if any(k=="sample" for k,_ in frames):
                        if stalled and recorder:recorder.event("resumed",{})
                        last=now; stalled=False
                    if recorder:
                        try:recorder.write(data,frames,decoder.bad)
                        except OSError as e:
                            self.events.put(("log_error",(p,str(e))))
                            recorder.close(); recorder=None
                    self.events.put(("frames",(p,frames,decoder.bad,decoder.missing,data)))
                if not stalled and now-last>.5:
                    stalled=True
                    if recorder:recorder.event("outage",{"timeout_ms":500})
        except (serial.SerialException,OSError) as e:
            if p.is_open:self.events.put(("error",str(e)))
        finally:
            if recorder:
                try:recorder.event("closed",{}); recorder.close()
                except OSError as e:self.events.put(("log_error",(p,str(e))))

    def _writer(self):
        while True:
            p,data,label=self.tx.get()
            try:
                with self.lock:
                    if p is not self.port or not p.is_open: raise serial.SerialException("串口已断开")
                    if p.write(data)!=len(data): raise serial.SerialException("串口数据未完整写入")
                self.log_tx.put((p,label)); self.events.put(("tx",label))
            except (serial.SerialException,OSError) as e: self.events.put(("error",str(e)))
    def _send(self,data,label):
        if not self.port or not self.port.is_open: messagebox.showinfo("串口未连接","请先连接 E35 对应 COM 口。",parent=self.root); return
        self.tx.put((self.port,data,label))
    def control(self,cmd):
        if cmd=="START" and self.start_mode.get()=="模块1：按PWM/时间":
            cmd="START_PWM"
        elif cmd=="START" and self.start_mode.get()=="模块4：按航向角/速度":
            cmd="START_CIRCLE"
        if cmd in ("START","START_PWM","START_CIRCLE") and self.latest_state!="ARMED":
            self.command_status.set("发车未发送：请先完成零偏标定并等待 ARMED")
            return
        if cmd=="START_CIRCLE":
            try:
                angle=int(self.circle_yaw.get());servo=int(self.circle_servo.get());a=float(self.circle_v1.get());b=float(self.circle_v2.get())
                v1=int(round(a*10));v2=int(round(b*10))
                if str(angle)!=self.circle_yaw.get().strip() or str(servo)!=self.circle_servo.get().strip() or not(5<=abs(angle)<=90 and 2<=servo<=15 and 2<=v1<=10 and 2<=v2<=10) or abs(a-v1/10)>1e-6 or abs(b-v2/10)>1e-6:raise ValueError
            except ValueError:
                self.command_status.set("发车未发送：航向±5–90°、舵机幅值2–15°、速度0.2–1.0m/s")
                return
            self.start.configure(state="disabled")
            self.pending_config=(f"{angle:+04d}",f"{servo:02d}",f"{v1:02d}",f"{v2:02d}")
            self.config_generation+=1
            self.command_status.set(f"等待小车确认航向 {angle}°、舵机 {servo}°、速度 {a:.1f}/{b:.1f} m/s")
            self._send(f"C4,{angle:+04d},{servo:02d},{v1:02d},{v2:02d}\r\n".encode("ascii"),f"C4 yaw={angle:+04d} servo={servo:02d} v={v1:02d}/{v2:02d} [CRLF]")
            self.root.after(2000,lambda generation=self.config_generation:self._config_timeout(generation))
            return
        if cmd=="START_PWM":
            try:
                pwm=int(self.pwm_percent.get()); hold=int(self.pwm_hold_s.get())
                if str(pwm)!=self.pwm_percent.get().strip() or str(hold)!=self.pwm_hold_s.get().strip() or not(10<=pwm<=40 and 2<=hold<=5):
                    raise ValueError
            except ValueError:
                self.command_status.set("发车未发送：模块1请选择整数 PWM 10–40% 和时间 2–5 秒")
                return
            self.start.configure(state="disabled")
            self.pending_config=(pwm,hold)
            self.config_generation+=1
            self.command_status.set(f"等待小车确认 PWM {pwm}%、保持 {hold} 秒")
            self._send(f"CONFIG,{pwm},{hold}\r\n".encode("ascii"),f"CONFIG PWM={pwm}% hold={hold}s [CRLF]")
            self.root.after(2000,lambda generation=self.config_generation:self._config_timeout(generation))
            return
        if cmd=="START":
            self.start.configure(state="disabled")
        if cmd=="DEBUG" and self.port and self.port.is_open:
            self.debug_status.set("按钮遥控：请求已发送，等待固件确认")
        if cmd=="STOP":
            self.pending_config=None
            self._deactivate_manual("按钮遥控：已发送停车命令")
        self._send((cmd+"\r\n").encode("ascii"),cmd+" [CRLF]")

    def _config_timeout(self,generation):
        if self.pending_config is not None and generation==self.config_generation:
            self.pending_config=None
            self.command_status.set("参数确认超时，未发送 START；检查串口及固件版本")
            self.start.configure(state="normal" if self.latest_state=="ARMED" else "disabled")

    def _deactivate_manual(self,status=None):
        self.manual_active=False
        self.manual_focus_lost=False
        for key in self.manual_keys:
            self.manual_keys[key]=False
            self.manual_sources[key]={"keyboard":False,"mouse":False}
        for button in self.joy_controls:button.configure(background="#e6eaf0",relief="raised")
        for button in self.joy_controls:button.configure(state="disabled")
        if status:self.debug_status.set(status)

    def _manual_command(self,focus_lost=False):
        if not self.manual_active or not self.port or not self.port.is_open:return
        up=bool(self.manual_keys["up"]); down=bool(self.manual_keys["down"])
        if down:up=False
        if focus_lost:up=False; down=True
        turn=(1 if self.manual_keys["left"] and not self.manual_keys["right"] else
              -1 if self.manual_keys["right"] and not self.manual_keys["left"] else 0)
        if focus_lost:turn=0
        payload=f"JOY,{int(up)},{int(down)},{turn}\r\n".encode("ascii")
        self._send(payload,f"JOY up={int(up)} down={int(down)} turn={turn}")

    def _manual_heartbeat(self):
        if self.manual_active:
            try:focused=self.root.focus_get() is not None
            except tk.TclError:focused=False
            if not focused and not self.manual_focus_lost:
                self.manual_focus_lost=True
                for key in self.manual_keys:
                    self.manual_keys[key]=False
                    self.manual_sources[key]={"keyboard":False,"mouse":False}
                for button in self.joy_controls:button.configure(background="#e6eaf0",relief="raised")
            elif focused:
                self.manual_focus_lost=False
            self._manual_command(focus_lost=not focused)
        self.root.after(50,self._manual_heartbeat)

    def start_calibration(self):
        if self.latest_state == "RUN":
            messagebox.showinfo("暂不可标定","小车处于 RUN 状态。请先停车，再启动 IMU 零偏标定。",parent=self.root); return
        self.start.configure(state="disabled")
        self.calibrate.configure(state="disabled")
        self.calibration.set("IMU 标定：命令已发送，等待固件确认")
        self._send(b"CALIBRATE\r\n","CALIBRATE [CRLF]")
    def send_custom(self):
        s=self.entry.get()
        if not s:return
        try:
            if self.hex_send.get():
                s=s.replace("0x","").replace("0X","").strip()
                if " " not in s and "\t" not in s:s=" ".join(s[i:i+2] for i in range(0,len(s),2))
                d=bytes.fromhex(s); label=" ".join(f"{x:02X}" for x in d)
            else:d=s.encode("utf-8"); label=s
            if self.crlf.get():d+=b"\r\n"; label+=" [CRLF]"
            self._send(d,label); self.entry.delete(0,"end")
        except ValueError as e:messagebox.showerror("HEX格式错误",f"例：01 03 A0 FF\n{e}",parent=self.root)
    def _consume(self,text):
        self.buf+=text; lines=self.buf.split("\n"); self.buf=lines.pop()
        if len(self.buf)>4096:self.buf=self.buf[-1024:]
        for raw in lines:
            line=raw.strip().lstrip("\ufeff")
            if not line:continue
            if line.startswith("CONFIG,OK,"):
                fields=line.split(",")
                if len(fields)==4 and self.pending_config is not None:
                    if (fields[2],fields[3])==tuple(map(str,self.pending_config)) and self.latest_state=="ARMED":
                        self.pending_config=None
                        self.command_status.set("参数回传一致，正在发送 START")
                        self._send(b"START\r\n","START after CONFIG ACK [CRLF]")
                    else:
                        self.pending_config=None
                        self.command_status.set("参数回传不一致或状态变化，未发车")
                continue
            if line.startswith("C4,OK,"):
                fields=line.split(",")
                if len(fields)==6 and self.pending_config is not None:
                    if tuple(fields[2:])==self.pending_config and self.latest_state=="ARMED":
                        self.pending_config=None
                        self.command_status.set("模块4参数回传一致，正在发送 START")
                        self._send(b"START\r\n","START after C4 ACK [CRLF]")
                    else:
                        self.pending_config=None
                        self.command_status.set("模块4回传不一致或状态变化，未发车")
                continue
            if line.startswith("RX,"):
                fields=line.split(",")
                if len(fields)==3 and fields[2]=="RECEIVED":
                    command=fields[1]
                    self.command_status.set(f"命令确认：固件已收到 {command}")
                    if command=="DEBUG":
                        self.debug_status.set("按钮遥控：命令已收到，等待执行确认")
                    elif command=="CALIBRATE":
                        self.calibration.set("IMU 标定：命令已收到，等待标定状态")
                elif len(fields)==3 and fields[1]=="START" and fields[2] in ("INVALID_PARAMS","WRONG_MODULE"):
                    self.pending_config=None
                    reason="参数超出范围或格式错误" if fields[2]=="INVALID_PARAMS" else "当前烧录固件不是模块1"
                    self.command_status.set("参数发车未执行："+reason)
                    self.start.configure(state="normal" if self.latest_state=="ARMED" else "disabled")
            elif line.startswith("CMD,"):
                self.summary.set("控制回执："+line[4:])
                if line.startswith("CMD,DEBUG,RUN,0"):
                    self.debug_active=True
                    self.manual_active=True
                    for key in self.manual_keys:
                        self.manual_keys[key]=False
                        self.manual_sources[key]={"keyboard":False,"mouse":False}
                    for button in self.joy_controls:button.configure(background="#e6eaf0",relief="raised")
                    for button in self.joy_controls:button.configure(state="normal")
                    self.debug.configure(state="disabled"); self.start.configure(state="disabled")
                    self.debug_status.set("按钮遥控运行：按住方向按钮操纵；STOP 可立即停车")
                    self._manual_command()
                elif line.startswith("CMD,START,RUN,0"):
                    self.start.configure(state="disabled")
                    self.calibration.set("IMU 标定：已完成；车辆正在运行")
                elif line.startswith("CMD,REJECTED_START,"):
                    self.start.configure(state="normal" if self.latest_state == "ARMED" else "disabled")
                    self.command_status.set("命令确认：发车被固件拒绝；需先完成 IMU 标定并等待 ARMED")
                elif line.startswith("CMD,REJECTED_DEBUG,"):
                    if not self.manual_active:
                        self._deactivate_manual("按钮遥控请求被拒绝："+line[4:]+"；请确认 ARMED，必要时先 STOP")
                elif line.startswith("CMD,STOP,") and self.debug_active:
                    self.debug_active=False
                    self._deactivate_manual("按钮遥控：已手动停车")
                    self.debug.configure(state="normal" if self.port and self.port.is_open else "disabled")
                    self.start.configure(state="disabled")
                if line.startswith("CMD,REJECTED_CALIBRATE,"):
                    self.calibration.set("IMU 标定：固件拒绝（RUN 中或正在标定）")
                    self.calibrate.configure(state="disabled" if self.latest_state == "RUN" else "normal")
            elif line.startswith("CALIBRATION,"):
                fields=line.split(",",2)
                if len(fields)==3:
                    phase,value=fields[1].upper(),fields[2]
                    if phase in ("IDLE","READY"):
                        self.calibration.set("IMU 标定：尚未标定，点击按钮开始")
                        self.calibrate.configure(state="disabled" if self.latest_state == "RUN" else ("normal" if self.port and self.port.is_open else "disabled"))
                    elif phase=="START":
                        self.calibration.set("IMU 标定：进行中 0%，请保持车体静止")
                        self.calibrate.configure(state="disabled")
                    elif phase=="PROGRESS":
                        self.calibration.set(f"IMU 标定：进行中 {value}%，请保持车体静止")
                    elif phase=="OK":
                        self.calibration.set("IMU 标定：完成，控制器即将进入 ARMED")
                        self.calibrate.configure(state="normal" if self.port and self.port.is_open else "disabled")
                    elif phase=="FAILED":
                        self.calibration.set("IMU 标定：失败（"+value+"），检查传感器并重试")
                        self.calibrate.configure(state="normal" if self.port and self.port.is_open and self.latest_state!="RUN" else "disabled")
                    elif phase=="BUSY":
                        self.calibration.set("IMU 标定：固件正在执行标定")
                        self.calibrate.configure(state="disabled")
            elif line.startswith("IMU_MODE,"):
                fields=line.split(",")
                if len(fields)>=3:
                    self.imu_mode=f"{fields[1]} / {fields[2]}"
                    self.summary.set(f"IMU 采集模式：{self.imu_mode}（无需 D1 中断）")
                else:self.summary.set("固件信息："+line)
            elif line.startswith("IMU_"):self.summary.set("固件信息："+line)
    def _reading(self,r):
        self.latest_state=r["state"]
        if self.latest_state=="ERROR" and self.debug_active:
            self.debug_active=False
            self._deactivate_manual(f"按钮遥控已停车：故障码 {r['error']}（含遥控心跳超时）")
            self.debug.configure(state="normal" if self.port and self.port.is_open else "disabled")
            self.start.configure(state="disabled")
        if self.port and self.port.is_open:
            self.start.configure(state="normal" if self.latest_state=="ARMED" and not self.debug_active and self.pending_config is None else "disabled")
        if self.latest_state=="RUN": self.calibrate.configure(state="disabled")
        elif self.latest_state=="ARMED" and self.calibration.get().startswith("IMU 标定：等待状态"):
            self.calibration.set("IMU 标定：已完成")
            self.calibrate.configure(state="normal" if self.port and self.port.is_open else "disabled")
        elif self.latest_state=="INIT" and self.calibration.get().startswith("IMU 标定：等待状态"):
            self.calibration.set("IMU 标定：尚未标定，点击按钮开始")
            self.calibrate.configure(state="normal" if self.port and self.port.is_open else "disabled")
        elif self.port and self.port.is_open and self.calibration.get().startswith("IMU 标定：尚未标定"):
            self.calibrate.configure(state="normal")
        if self.imu_mode == "未收到模式声明":
            self.imu_mode = "遥测有效 / 启动声明未收到"
        vals=tuple(f"{r[k]:.3f}" if isinstance(r[k],float) else r[k] for k in self.reading_cols)
        if self.tree.exists("latest"):self.tree.item("latest",values=vals)
        else:self.tree.insert("","end",iid="latest",values=vals)
        error_name={0:"正常",1:"IMU 未标定",2:"发车时车轮在运动",3:"IMU 数据超时",4:"运行后轮速过低/堵转",5:"运行超时",6:"控制周期异常",7:"串口 DMA 故障",8:"串口接收错误",9:"遥控心跳超时"}.get(r['error'],"未知错误")
        self.summary.set(f"已解析 {self.count} 帧 · {r['state']} / 错误 {r['error']}（{error_name}） · 丢失样本 {self.missing_frames} · 无效帧 {self.bad_frames} · 日志段 {r['segment']} · 舵机为指令偏移角，非实测前轮角")
    def _log(self,s,tag=""):
        self.log.configure(state="normal"); self.log.insert("end",s,tag); n=int(self.log.index("end-1c").split(".")[0])
        if n>1000:self.log.delete("1.0",f"{n-999}.0")
        self.log.see("end"); self.log.configure(state="disabled")
    def _poll(self):
        deadline=time.monotonic()+.01
        for _ in range(256):
            if time.monotonic()>deadline:break
            try:k,v=self.events.get_nowait()
            except queue.Empty:break
            if k=="frames":
                p,frames,bad,missing,raw=v
                if p is not self.port:continue
                self.bad_frames=bad; self.missing_frames=missing
                if self.hex_view.get():self._log("RX: "+raw.hex(" ")+"\n")
                for kind,value in frames:
                    if kind=="sample":
                        self.pending_reading=value; self.count+=1; self.last_valid=time.monotonic()
                    elif kind=="event":self._consume(value); self._log(value,"info")
                    else:self._log("DIAG: "+str(value)+"\n","info")
            elif k in ("log_path","log_error"):
                p,value=v
                if p is self.port:
                    self.log_status.set(("正在保存完整日志：" if k=="log_path" else "日志写入失败：")+value)
            elif k=="tx":self._log("TX: "+str(v)+"\n","tx")
            else:self._log("串口错误："+str(v)+"\n","info"); self.status.set("串口错误，请检查连接")
        now=time.monotonic()
        if self.pending_reading is not None and now-self.last_draw>=1/30:
            self._reading(self.pending_reading); self.pending_reading=None; self.last_draw=now
        if self.port and self.port.is_open:
            if self.last_valid and now-self.last_valid<=.5:
                self.status.set(f"已连接 {self.port.port} · 有效数据接收中")
            else:self.status.set("数据中断 / 等待有效帧；显示值已过期")
        self.root.after(20,self._poll)
    def clear(self):
        self.log.configure(state="normal"); self.log.delete("1.0","end"); self.log.configure(state="disabled")
        for x in self.tree.get_children():self.tree.delete(x)
        self.count=0; self.pending_reading=None; self.imu_mode="未收到模式声明"; self.summary.set("已清空；等待协议 v1 二进制遥测数据…")
    def _disconnect(self):
        self.pending_config=None
        self.debug.configure(state="disabled"); self.debug_active=False; self._deactivate_manual("按钮遥控：串口已断开；固件心跳超时后停车")
        with self.lock:
            p=self.port; self.port=None
            if p and p.is_open:p.close()
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
            self.reader_thread=None
        self.pending_reading=None; self.last_valid=0
        self.status.set("未连接"); self.command_status.set("命令确认：等待上位机命令"); self.conn_btn.configure(text="连接串口"); self.start.configure(state="disabled"); self.stop.configure(state="disabled"); self.calibrate.configure(state="disabled"); self.calibration.set("IMU 标定：连接串口后可启动"); self.ports.configure(state="readonly"); self.baudbox.configure(state="readonly"); self._log("[串口已断开]\n","info")
    def close_app(self):
        if self.port:self._disconnect()
        self.root.destroy()

def main():
    root=tk.Tk(); E35App(root); root.mainloop()
if __name__=="__main__":main()
