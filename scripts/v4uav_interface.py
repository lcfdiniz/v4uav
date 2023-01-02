#!/usr/bin/env python
import rospy
from v4uav.msg import v4uav_command
from v4uav.msg import v4uav_setpoint
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import os

class ToolbarApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        # Step 1: Frame configuration
        self.parent = parent # MainApp frame
        self.config(background='#ffffff')
        logo_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'assets/images/v4uav_logo.png'))
        self.logo = tk.PhotoImage(file=logo_path).subsample(10,10)
        tk.Label(self, image=self.logo, bg='#ffffff').pack(side='left')
        tk.Label(self, text='V4UAV', font=('TkDefaultFont', 14,'bold'), fg='#3e3e3e', bg='#ffffff').pack(side='left', pady=10, padx=10)
class CommanderApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        # Step 1: Frame configuration
        self.parent = parent # MainApp frame
        tk.Label(self, text='Commander', font=('TkDefaultFont', 12,'bold'), fg='white', bg='#02577a').pack(pady=10)
        self.config(background='#02577a')
        self.n_inputs = {'TAKEOFF': 1, 'SET_POS': 4, 'SET_REL_POS': 4, 'HOLD': 0, 
                        'GET_POS': 0, 'TRACKING': 0, 'LANDING': 0, 'MANUAL': 0, 'RTL': 0}
        # Step 2: Structuring layout
        self.inputs_box = tk.Frame(self)
        self.inputs_box.config(background='#02577a')
        # Step 3: Adding widgets
        self.combobox = ttk.Combobox(self.inputs_box)
        self.combobox['values'] = ('TAKEOFF', 'SET_POS', 'SET_REL_POS', 'HOLD', 
                                    'GET_POS', 'TRACKING', 'LANDING', 'MANUAL', 'RTL')
        self.combobox['state'] = 'readonly'
        self.combobox.bind('<<ComboboxSelected>>', self.enable_inputs)
        tk.Label(self.inputs_box, text="V4UAV mode:", fg='white', bg='#02577a').pack()
        self.combobox.pack()
        self.inputs = []
        for i in range(0,4):
            tk.Label(self.inputs_box, text="Input " + str(i+1) + ":", fg='white', bg='#02577a').pack()
            self.inputs.append(tk.Entry(self.inputs_box, state='disabled'))
            self.inputs[i].pack()
        self.send_msg_btn = tk.Button(self.inputs_box, text='Send message', command=self.fill_msg)
        self.send_msg_btn.pack(pady=10)
        # Packing frame
        self.inputs_box.pack(pady=10, fill='x')
    
    def enable_inputs(self, event):
        mode = self.combobox.get()
        for i in range(0, self.n_inputs[mode]):
            self.inputs[i].configure(state='normal')
        for i in reversed(range(self.n_inputs[mode], 4)):
            self.inputs[i].configure(state='disabled')
    
    def fill_msg(self, event=None):
        mode = self.combobox.get()
        if mode != '':
            self.command = [mode]
            try:
                for i in range(0, self.n_inputs[mode]):
                    self.command.append(float(self.inputs[i].get()))
                for i in reversed(range(self.n_inputs[mode], 4)):
                    self.command.append(0.0)
                self.pub_msg()
            except ValueError:
                messagebox.showerror(title='Invalid input', message='Please ensure inputs are numeric.')
        else:
            messagebox.showerror(title='Invalid input', message='Please select a valid V4UAV mode.')

    def pub_msg(self):
        msg = v4uav_command()
        msg.mode = self.command[0]
        msg.input_1 = self.command[1]
        msg.input_2 = self.command[2]
        msg.input_3 = self.command[3]
        msg.input_4 = self.command[4]
        rate = rospy.Rate(20)
        command_pub.publish(msg)
        rate.sleep()

class ControllerApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        # Step 1: Frame configuration
        self.parent = parent # MainApp frame
        tk.Label(self, text='Controller', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7').pack(pady=10)
        self.config(background='#02a9f7')
        # Step 2: Structuring layout
        # The Controller frame has 2 major portions: xy_frame (top) and zyaw_frame (bottom)
        self.xy_frame = tk.Frame(self)
        self.xy_frame.config(background='#02a9f7')
        self.zyaw_frame = tk.Frame(self)
        self.zyaw_frame.config(background='#02a9f7')
        # Step 3: Adding widgets
        self.slider_x = tk.Scale(self.xy_frame, from_=1.0, to=-1.0, resolution=0.1, orient='vertical', 
                                label='X', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0, 
                                command=self.send_msg)
        self.slider_x.grid(row=0, column=0, padx=30)
        self.slider_y = tk.Scale(self.xy_frame, from_=-1.0, to=1.0, resolution=0.1, orient='horizontal', 
                                label='Y', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0,
                                command=self.send_msg)
        self.slider_y.grid(row=0, column=1, padx=30)
        self.slider_z = tk.Scale(self.zyaw_frame, from_=1.0, to=-1.0, resolution=0.1, orient='vertical', 
                                label='Z', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0,
                                command=self.send_msg)
        self.slider_z.grid(row=0, column=0, padx=30)
        self.slider_yaw = tk.Scale(self.zyaw_frame, from_=-1.0, to=1.0, resolution=0.1, orient='horizontal',
                                label='Yaw', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0,
                                command=self.send_msg)
        self.slider_yaw.grid(row=0, column=1, padx=30)
        # Packing frames
        self.xy_frame.pack(pady=20)
        self.zyaw_frame.pack()
    
    def send_msg(self, event):
        msg = v4uav_setpoint()
        msg.vx_sp = self.slider_x.get()*max_vx
        msg.vy_sp = self.slider_y.get()*max_vy
        msg.vz_sp = self.slider_z.get()*max_vz
        msg.yaw_rate_sp = self.slider_yaw.get()*max_yaw_rate
        rate = rospy.Rate(20)
        input_pub.publish(msg)
        rate.sleep()
class MainApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        # Step 1: Root configuration
        self.parent = parent # root
        self.parent.title('V4UAV')
        self.parent.geometry("800x400")
        # Step 2: Setting the icon
        icon_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'assets/images/v4uav_icon.png'))
        self.icon = tk.PhotoImage(file=icon_path)
        self.parent.tk.call('wm', 'iconphoto', self.parent._w, self.icon)
        # Step 3: Creating the menu bar
        self.menubar = tk.Menu(self.parent)
        self.filemenu = tk.Menu(self.menubar, tearoff=0) # File menu
        self.filemenu.add_command(label='Close', command=exit)
        self.menubar.add_cascade(menu=self.filemenu, label='File')
        self.helpmenu = tk.Menu(self.menubar, tearoff=0) # Help menu
        self.helpmenu.add_command(label='About', command=self.about)
        self.menubar.add_cascade(menu=self.helpmenu, label='Help')
        self.parent.configure(menu=self.menubar)
        # Step 4: Structuring layout
        # The interface has 2 major portions: Commander and Controller
        self.commander = CommanderApp(self)
        self.controller = ControllerApp(self)
        # The toolbar only displays an image
        self.toolbar = ToolbarApp(self)
        # Packing the frames
        self.toolbar.pack(side='top', fill='x')
        self.commander.pack(side='left', fill='both', expand=True)
        self.controller.pack(side='left', fill='both', expand=True)
        # Step 5: Bind scales with keyboard
        self.parent.bind("<w>", lambda e: self.controller.slider_x.set(self.controller.slider_x.get()+0.1))
        self.parent.bind("<W>", lambda e: self.controller.slider_x.set(self.controller.slider_x.get()+0.1))
        self.parent.bind("<s>", lambda e: self.controller.slider_x.set(self.controller.slider_x.get()-0.1))
        self.parent.bind("<S>", lambda e: self.controller.slider_x.set(self.controller.slider_x.get()-0.1))
        self.parent.bind("<d>", lambda e: self.controller.slider_y.set(self.controller.slider_y.get()+0.1))
        self.parent.bind("<D>", lambda e: self.controller.slider_y.set(self.controller.slider_y.get()+0.1))
        self.parent.bind("<a>", lambda e: self.controller.slider_y.set(self.controller.slider_y.get()-0.1))
        self.parent.bind("<A>", lambda e: self.controller.slider_y.set(self.controller.slider_y.get()-0.1))
        self.parent.bind("<i>", lambda e: self.controller.slider_z.set(self.controller.slider_z.get()+0.1))
        self.parent.bind("<I>", lambda e: self.controller.slider_z.set(self.controller.slider_z.get()+0.1))
        self.parent.bind("<k>", lambda e: self.controller.slider_z.set(self.controller.slider_z.get()-0.1))
        self.parent.bind("<K>", lambda e: self.controller.slider_z.set(self.controller.slider_z.get()-0.1))
        self.parent.bind("<l>", lambda e: self.controller.slider_yaw.set(self.controller.slider_yaw.get()-0.1))
        self.parent.bind("<L>", lambda e: self.controller.slider_yaw.set(self.controller.slider_yaw.get()-0.1))
        self.parent.bind("<j>", lambda e: self.controller.slider_yaw.set(self.controller.slider_yaw.get()+0.1))
        self.parent.bind("<J>", lambda e: self.controller.slider_yaw.set(self.controller.slider_yaw.get()+0.1))
        self.parent.bind("<Return>", self.commander.fill_msg)
    
    def about(self):
        win = tk.Toplevel()
        win.title("About")
        win.tk.call('wm', 'iconphoto', win._w, self.icon)
        text = """V4UAV is a ROS package to control UAVs in 
        power transmission line inspection task 
        using computer vision techniques. 
        It is the master thesis project of Lucas F. Diniz 
        (lcfdiniz@outlook.com), also the developer and mantainer. 
        For more information, please refer to the 
        GitHub repo (https://github.com/lcfdiniz/V4UAV)."""
        tk.Label(win, text=text).pack(padx=20, pady=20)
        tk.Button(win, text='OK', command=win.destroy).pack(pady=10)

def interface():
    rospy.init_node('v4uav_interface')
    # Step 1: Publish to MAVROS topics
    global command_pub, input_pub
    command_pub = rospy.Publisher("/v4uav/command", v4uav_command, queue_size=10)
    input_pub = rospy.Publisher("/v4uav/input", v4uav_setpoint, queue_size=10)
    # Step 2: Declare global variables
    global max_vx, max_vy, max_vz, max_yaw_rate
    max_vx = rospy.get_param('/interface/max_vx')
    max_vy = rospy.get_param('/interface/max_vy')
    max_vz = rospy.get_param('/interface/max_vz')
    max_yaw_rate = rospy.get_param('/interface/max_yaw_rate')
    # Step 3: Creating Tkinter interface
    root = tk.Tk()
    MainApp(root).pack(side="top", fill="both", expand=True)
    root.mainloop() # We don't need rospy.spin() since root.mainloop() will keep the program running

if __name__ == '__main__':
    try:
        interface()
    except rospy.ROSInterruptException:
        pass