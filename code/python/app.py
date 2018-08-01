import gimbal as gb
import serial
import serial.tools.list_ports as lp
import tkinter as tk
from tkinter import ttk, messagebox

LARGE_FONT = ("Verdana", 12)
FILL_HOZ = tk.W+tk.E+tk.N+tk.S


class gimbalApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        #--------------------tk setup--------------------
        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, "DDC Gimbal Controller")
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0,weight=1)
        container.grid_columnconfigure(0,weight=1)
        
        #------------------frame setup-------------------
        self.frames = {}
        self.frame = StartPage(container,self)
        frames = [StartPage]
        #for F in frames:
        #    frame = F(container,self)
        #    self.frames[F] = frame
        self.frame.grid(row=0,column=0, sticky="nswe")
        self.frame.tkraise()
        self.frame.disable_buttons()

        #-----------------menubar setup------------------
        menubar = tk.Menu(container)
        
        self.connmenu = tk.Menu(menubar, tearoff=0)        
        self.connmenu.add_cascade(label="COM Port")
        self.connmenu.add_command(label="Connect", command=lambda:self._connect_gimbal())
        
        self.connmenu.add_separator()
        self.connmenu.add_command(label="Scan", command=lambda:self._refresh_ports(menubar))
        
        menubar.add_cascade(label="Connect", menu=self.connmenu)

        tk.Tk.config(self, menu=menubar)
        self.connmenu.entryconfig(1, state="disabled")

        self._refresh_ports(menubar)

    def disconnect(self):
        try:
            self.gimbal.close()
        except Exception as e:
            pass
        self.frame.disconnect()
        self.frame.update()
        self.connmenu.entryconfig(0, state="normal")
        self.connmenu.entryconfig(1, state="disabled")
        self.connmenu.entryconfig(3, label="Scan")

    def _refresh_ports(self,root):
        self.disconnect()
        comports = lp.comports()
        self.ports = []
        portmenu = tk.Menu(root, tearoff=0)
        for i in range(len(comports)):
            self.ports.append(comports[i].device)
            portmenu.add_command(label=self.ports[i],
                                 command=lambda i=i:self._connect_command(index=i))

        self.connmenu.entryconfig(0, menu=portmenu)
        

    def _connect_command(self, index):
        print(index)
        self.connmenu.entryconfig(1, state="normal")
        self.select = index

    def _connect_gimbal(self):
        try:
            self.frame.update_status("connecting")
            self.frame.update()
            print('Trying to connect')
            self.gimbal = gb.Gimbal(self.ports[self.select])
        except Exception as e:
            print(str(e))
            tk.messagebox.showerror("Connection Error", "Could not connect to gimbal")
            self.disconnect()
        else:
            print('hi')
            self.frame.enable_buttons()
            
            self.connmenu.entryconfig(0, state="disabled")
            self.connmenu.entryconfig(1, state="disabled")
            self.connmenu.entryconfig(3, label="Disconnect")
            self.frame.update_status("connected")

    def gimbal_power(self, value):
        if value == 0 or value == 1:
            self.gimbal.set_enable(value)

    def gimbal_direction(self, value):
        if value == 0 or value == 1:
            try:
                self.gimbal.set_direction(value)
            except serial.serialutil.SerialException:
                self.disconnect()
                tk.messagebox.showerror("Connection Error", "Gimbal disconnected")
    def gimbal_position(self, value):
        self.gimbal.set_position(value)


class StartPage(tk.Frame):
    def __init__(self, parent, controller):
        self._row_count = 0
        
        tk.Frame.__init__(self,parent)

        status_frame = self._add_labelframe(self,"", 0, self._get_row(), 2)
        
        label = tk.Label(status_frame, text="Status:", font=LARGE_FONT)
        label.grid(column=0, row=0, pady=10, padx=10, sticky=tk.E)

        self.status_label = tk.Label(status_frame, font=LARGE_FONT, text="Disconnected", foreground = "red")
        self.status_label.grid(column=1, row=0, pady=10, padx=10, sticky=tk.W)

        data_frame = self._add_labelframe(self, "Gimbal Data", 0, self._get_row(), 2)

        testlabel = tk.Label(data_frame, text="Test", font=LARGE_FONT)
        testlabel.grid(column=0, row=0, padx=10, sticky=FILL_HOZ)

        entry_text = tk.StringVar()
        testentry = tk.Entry(data_frame, state='disabled', textvariable = entry_text, font=LARGE_FONT, width=10)
        testentry.grid(column=1, row=0, padx=10, pady=5, sticky=FILL_HOZ)
        entry_text.set("hello")

        testlabel = tk.Label(data_frame, text="Test", font=LARGE_FONT)
        testlabel.grid(column=2, row=0, padx=10, sticky=FILL_HOZ)

        entry2_text = tk.StringVar()
        testentry2 = tk.Entry(data_frame, state='disabled', textvariable = entry2_text, font=LARGE_FONT, width=10)
        testentry2.grid(column=3, row=0, padx=10, pady=5, sticky=FILL_HOZ)
        entry2_text.set("hi")
        
        movement_frame = self._add_labelframe(self, "Gimbal Movement", 0, self._get_row(), 1)

        self.buttons = []
        
        self.enable = tk.IntVar()
        self._add_radiobtn("Enable", movement_frame, self.enable, 1,
                           lambda:self._power_btn_resp(1, controller), 0, 0)
        self._add_radiobtn("Disable", movement_frame, self.enable, 0,
                           lambda:self._power_btn_resp(0, controller), 1, 0)


        direction_frame = self._add_labelframe(self, "Gimbal Direction", 0, self._get_row(), 1)
        
        self.direction = tk.IntVar()
        self._add_radiobtn("Normal", direction_frame, self.direction, 1,
                           lambda:controller.gimbal_direction(1),0,0)
        self._add_radiobtn("Inverted", direction_frame, self.direction, 0,
                           lambda:controller.gimbal_direction(0),1,0)

        self.enable.set(2)
        self.direction.set(2)
        
        position_frame = self._add_labelframe(self, "Gimbal Position", 1, 2, 1)

        self.position = tk.IntVar()
        self.slider = tk.Scale(position_frame, from_=0, to=180, orient=tk.HORIZONTAL,var=self.position,
                          command=lambda i = self.position: self._position_resp(i, controller))
        self.position.set(90)
        self.slider.grid(column=0, row=0, padx=10, sticky=FILL_HOZ)

    def _get_row(self):
        self._row_count += 1
        return self._row_count - 1
    
    def _add_radiobtn(self, text, frame, var, val, cmd, col, row):
        btn = tk.Radiobutton(frame, font=LARGE_FONT, text=text, variable=var, value=val, command=cmd)
        btn.grid(column=col, row=row, padx=10, sticky=FILL_HOZ)
        self.buttons.append(btn)

    def _add_labelframe(self, root, text, col, row, col_span = 1):
        label_frame = tk.LabelFrame(root, text=text, font=LARGE_FONT)
        label_frame.grid(column=col, row=row, padx=10, pady=5, sticky=FILL_HOZ, columnspan = col_span)
        return label_frame

    def _position_toggle(self, val):
        if val == 0:
            self.position.set(90)
            self.slider.configure(state="disabled")
        elif val == 1:
            self.slider.configure(state="normal")

    def _position_resp(self, val, controller):
        try:
            controller.gimbal_position(int(val))
        except AttributeError:
            self._position_toggle(0)
        except serial.serialutil.SerialException:
            controller.disconnect()
            tk.messagebox.showerror("Connection Error", "Gimbal disconnected")
        
    def _power_btn_resp(self, val, controller):
        if val == 0:
            self._position_toggle(1)
        elif val == 1:
            self._position_toggle(0)
        else:
            return
        controller.gimbal_power(val)
        
    def enable_buttons(self):
        for button in self.buttons:
            button['state'] = 'normal'

    def disable_buttons(self):
        for button in self.buttons:
            button['state'] = 'disabled'
        self.enable.set(3)
        self.direction.set(3)

    def disconnect(self):
        self.update_status("disconnected")
        self.disable_buttons()
        self._position_toggle(0)
        
    def update_status(self, status):
        if status == "disconnected":
            self.status_label.configure(text="Disconnected", foreground="red")
        elif status == "connecting":
            self.status_label.configure(text="Connecting", foreground="orange")
        elif status == "connected":
            self.status_label.configure(text="Connected", foreground="green")

app = gimbalApp()
