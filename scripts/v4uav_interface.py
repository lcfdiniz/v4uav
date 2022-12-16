#!/usr/bin/env python
import rospy
from v4uav.msg import v4uav_command
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import os

class CommanderApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent
        # Step 1: Frame configuration
        tk.Label(self, text='Commander', font=('TkDefaultFont', 12,'bold'), fg='white', bg='#02577a').pack()
        self.config(background='#02577a')
        self.n_inputs = {'TAKEOFF': 1, 'SET_POS': 4, 'SET_REL_POS': 4, 'HOLD': 0, 
                        'GET_POS': 0, 'TRACKING': 0, 'LANDING': 0, 'RTL': 0}
        # Step 2: # Step 2: Organizing layout with (child) frames (inputs_box)
        self.inputs_box = tk.Frame(self)
        self.inputs_box.config(background='#02577a')
        # Step 3: Adding widgets
        self.combobox = ttk.Combobox(self.inputs_box)
        self.combobox['values'] = ('TAKEOFF', 'SET_POS', 'SET_REL_POS', 'HOLD', 
                                    'GET_POS', 'TRACKING', 'LANDING', 'RTL')
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
        self.inputs_box.pack(pady=50, fill='x')
    
    def enable_inputs(self, event):
        mode = self.combobox.get()
        for i in range(0, self.n_inputs[mode]):
            self.inputs[i].configure(state='normal')
        for i in reversed(range(self.n_inputs[mode], 4)):
            self.inputs[i].configure(state='disabled')
    
    def fill_msg(self):
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
        v4uav_pub.publish(msg)
        rate.sleep()

class ControllerApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        # Step 1: Frame configuration
        self.parent = parent
        self.label = tk.Label(self, text='Controller', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7').pack()
        self.config(background='#02a9f7')
        # Step 2: Organizing layout with (children) frames (top and bottom)
        self.top = tk.Frame(self)
        self.top.config(background='#02a9f7')
        self.bottom = tk.Frame(self)
        self.bottom.config(background='#02a9f7')
        # Step 3: Adding widgets
        self.slider_x = tk.Scale(self.top, from_=1.0, to=-1.0, resolution=0.1, orient='vertical', label='X', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0)
        self.slider_x.grid(row=0, column=0, padx=30)
        self.slider_y = tk.Scale(self.top, from_=-1.0, to=1.0, resolution=0.1, orient='horizontal', label='Y', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0)
        self.slider_y.grid(row=0, column=1, padx=30)
        self.slider_z = tk.Scale(self.bottom, from_=1.0, to=-1.0, resolution=0.1, orient='vertical', label='Z', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0)
        self.slider_z.grid(row=0, column=0, padx=30)
        self.slider_yaw = tk.Scale(self.bottom, from_=-1.0, to=1.0, resolution=0.1, orient='horizontal', label='Yaw', font=('TkDefaultFont', 12,'bold'), fg='#3e3e3e', bg='#02a9f7', highlightthickness=0)
        self.slider_yaw.grid(row=0, column=1, padx=30)
        # Packing frames
        self.top.pack(pady=50)
        self.bottom.pack()
        # Step 3: Biding

class MainApp(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        # Step 1: Root configuration
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent
        self.parent.title('V4UAV')
        self.parent.geometry("800x400")
        # Step 2: Setting the icon
        icon_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'assets/v4uav.png'))
        icon = tk.PhotoImage(file=icon_path)
        self.parent.tk.call('wm', 'iconphoto', self.parent._w, icon)
        # Step 3: Creating the menu bar
        self.menubar = tk.Menu(self.parent)
        # File menu
        self.filemenu = tk.Menu(self.menubar, tearoff=0)
        self.filemenu.add_command(label='Close', command=exit)
        self.menubar.add_cascade(menu=self.filemenu, label='File')
        # Help menu
        self.helpmenu = tk.Menu(self.menubar, tearoff=0)
        self.helpmenu.add_command(label='About', command=exit)
        self.menubar.add_cascade(menu=self.helpmenu, label='Help')
        # Configuring menubar
        self.parent.configure(menu=self.menubar)
        # Step 4: Organizing layout with (children) frames (Commander and Controller)
        # The interface has 2 major portions: Commander and Controller
        self.commander = CommanderApp(self)
        self.controller = ControllerApp(self)
        # Packing the frames
        self.commander.pack(side='left', fill='both', expand=True)
        self.controller.pack(side='left', fill='both', expand=True)

def interface():
    rospy.init_node('v4uav_interface')
    # Step 1: Publish to a MAVROS topic
    global v4uav_pub
    v4uav_pub = rospy.Publisher("/v4uav/command", v4uav_command, queue_size=10)
    # Step 2: Creating Tkinter interface
    root = tk.Tk()
    MainApp(root).pack(side="top", fill="both", expand=True)
    root.mainloop() # We don't need rospy.spin() since root.mainloop() will keep the program running

if __name__ == '__main__':
    try:
        interface()
    except rospy.ROSInterruptException:
        pass