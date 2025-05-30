#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,String
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from robot_motion_service.srv import SetPosition, SolveIK
import random
import paho.mqtt.client as mqtt


# MQTT Broker details
# BROKER = "test.mosquitto.org"
# PORT = 1883

class RobotControlUI:
    def switch_controllers(self):
        import subprocess
        cmd = ["ros2", "control", "switch_controllers", "--activate", "joint_state_broadcaster", "--deactivate", "joint_trajectory_controller", "--activate", "velocity_controller"]
        subprocess.run(cmd)
        
    def __init__(self, root, ros_node):
        self.style = ttk.Style()
        self.root = root
        self.ros_node = ros_node
        self.velocity = 0.5  # Default velocity
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint = self.joint_names[0]
        self.control_mode = "jog"  # Modes: 'jog', 'velocity', or 'cartesian'
        self.client = self.ros_node.create_client(SetPosition, '/set_position')
        self.solve_ik_client = self.ros_node.create_client(SolveIK, '/solve_ik')

        # กำหนดค่า joint_values เริ่มต้นที่นี่ เพื่อให้มีค่าอยู่ก่อนที่จะสร้าง UI
        self.joint_values = {joint: 0.0 for joint in self.joint_names}
        
        self.mqtt_toggle_state = False  # MQTT toggle state
        self.current_mode = None  # Default mode (None)
        self.mqtt_client = None  # MQTT Client instance
        self.robot_name = "fibotx3"  # Change this if needed
        self.button_styles = {}  # เก็บสีเดิมของปุ่ม

        self.cartesian_entries = {}
        self.cartesian_labels = {}
        
        # Initialize kinematics for cartesian control
        self.kinematics = None
        
        # Default cartesian values - adjusted to be within workspace
        self.cartesian_values = {
            'x': 100.0,
            'y': 100.0,
            'z': 250.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }

        self.publisher = self.ros_node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.control_mode_publisher = self.ros_node.create_publisher(String, '/control_mode/state', 10)  # New publisher

        self.root.geometry("1200x800")
        self.root.attributes('-fullscreen', False)

        # Initialize kinematics
        self.init_kinematics()
        
        # Create the UI
        self.create_ui()
    def get_mqtt_config(self):
        """ดึงค่า MQTT Configuration จาก UI"""
        try:
            broker = self.broker_entry.get().strip()
            port = int(self.port_entry.get().strip())
            
            # ตรวจสอบค่าว่าง
            if not broker:
                broker = "test.mosquitto.org"
            if port <= 0 or port > 65535:
                port = 1883
                
            return broker, port
        except ValueError:
            # ถ้า port ไม่ใช่ตัวเลข ใช้ค่าเริ่มต้น
            self.ros_node.get_logger().warning("Invalid port number, using default 1883")
            return self.broker_entry.get().strip() or "test.mosquitto.org", 1883

    def test_mqtt_connection(self):
        """ทดสอบการเชื่อมต่อ MQTT Broker"""
        broker, port = self.get_mqtt_config()
        
        self.connection_status_label.config(text="Testing connection...", bootstyle="warning")
        self.root.update()
        
        try:
            # สร้าง MQTT client ชั่วคราวเพื่อทดสอบ
            test_client = mqtt.Client()
            test_client.on_connect = self.on_test_connect
            test_client.on_disconnect = self.on_test_disconnect
            
            # ตั้งค่า timeout
            test_client.connect(broker, port, 10)
            test_client.loop_start()
            
            # รอผลลัพธ์ 3 วินาที
            self.root.after(3000, lambda: self.finish_connection_test(test_client))
            
            self.ros_node.get_logger().info(f"Testing connection to {broker}:{port}")
            
        except Exception as e:
            self.connection_status_label.config(
                text=f"Connection Failed: {str(e)[:30]}...", 
                bootstyle="danger"
            )
            self.ros_node.get_logger().error(f"MQTT Test Connection Failed: {e}")

    def on_test_connect(self, client, userdata, flags, rc):
        """Callback สำหรับการทดสอบการเชื่อมต่อ"""
        if rc == 0:
            self.connection_status_label.config(text="✅ Connection Successful!", bootstyle="success")
            self.ros_node.get_logger().info("MQTT Test Connection: SUCCESS")
        else:
            self.connection_status_label.config(text=f"❌ Connection Failed (Code: {rc})", bootstyle="danger")
            self.ros_node.get_logger().error(f"MQTT Test Connection Failed with code: {rc}")

    def on_test_disconnect(self, client, userdata, rc):
        """Callback เมื่อการทดสอบเสร็จสิ้น"""
        self.ros_node.get_logger().info("MQTT Test Connection: Disconnected")

    def finish_connection_test(self, test_client):
        """จบการทดสอบการเชื่อมต่อ"""
        try:
            test_client.loop_stop()
            test_client.disconnect()
        except:
            pass
    def init_kinematics(self):
        """Initialize kinematics - minimal version"""
        try:
            import os
            import sys
            
            # เพิ่ม path
            script_dir = os.path.dirname(os.path.abspath(__file__))
            if script_dir not in sys.path:
                sys.path.append(script_dir)
            
            # Import และใช้งาน
            from kinematic_fibox import FiboX_Borot
            self.kinematics = FiboX_Borot()
            self.ros_node.get_logger().info("✅ Kinematics loaded successfully!")
            
        except Exception as e:
            self.ros_node.get_logger().error(f"❌ Failed to load kinematics: {e}")
            self.kinematics = None     
    def create_ui(self):
        self.switch_controllers()
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Create a frame for mode selection buttons
        mode_frame = ttk.Frame(self.root)
        mode_frame.pack(pady=20)
        
        # Create buttons for each mode
        self.jog_mode_button = ttk.Button(mode_frame, text="Jog Mode", 
                                     bootstyle="primary" if self.control_mode == "jog" else "secondary", 
                                     command=lambda: self.set_mode("jog"), 
                                     width=30, padding=30)
        self.jog_mode_button.pack(side="left", padx=10)
        
        self.velocity_mode_button = ttk.Button(mode_frame, text="Velocity Mode", 
                                          bootstyle="primary" if self.control_mode == "velocity" else "secondary", 
                                          command=lambda: self.set_mode("velocity"), 
                                          width=30, padding=30)
        self.velocity_mode_button.pack(side="left", padx=10)
        
        self.cartesian_mode_button = ttk.Button(mode_frame, text="Cartesian Mode", 
                                           bootstyle="primary" if self.control_mode == "cartesian" else "secondary", 
                                           command=lambda: self.set_mode("cartesian"), 
                                           width=30, padding=30)
        self.cartesian_mode_button.pack(side="left", padx=10)

        if self.control_mode == "jog":
            self.create_jog_ui()
        elif self.control_mode == "velocity":
            self.create_velocity_ui()
        else:
            self.create_cartesian_ui()
    
    def create_jog_ui(self):
        # ตรวจสอบว่ามีค่า joint_values อยู่แล้วหรือไม่ ถ้าไม่มีจึงสร้างขึ้นใหม่
        if not hasattr(self, 'joint_values'):
            self.joint_values = {joint: 0.0 for joint in self.joint_names}
            
        ttk.Label(self.root, text="Select Joint", font=("Arial", 36)).pack(pady=20)
        self.joint_selector = ttk.Combobox(self.root, values=self.joint_names, state="readonly", font=("Arial", 44), width=30, postcommand=lambda: self.joint_selector.configure(height=10))
        self.joint_selector.bind('<Up>', lambda e: self.navigate_joint_selection(-1))
        self.joint_selector.bind('<Down>', lambda e: self.navigate_joint_selection(1))
        self.style.configure('TCombobox', font=('Arial', 44))
        self.style.configure('TCombobox.Listbox', font=('Arial', 44))
        self.root.option_add('*TCombobox*Listbox*Font', ('Arial', 44))
        self.root.option_add('*TCombobox*Listbox*width', 30)
        self.root.option_add('*TCombobox*Listbox.font', ('Arial', 44))
        self.joint_selector.option_add('*TCombobox*Listbox*Font', ('Arial', 44))
        self.joint_selector.option_add('*TCombobox*Listbox.font', ('Arial', 44))
        self.joint_selector.pack(pady=20)
        self.joint_selector.set(self.current_joint)

        ttk.Label(self.root, text="Velocity", font=("Arial", 36)).pack(pady=20)
        
        # Create a frame for velocity controls
        velocity_frame = ttk.Frame(self.root)
        velocity_frame.pack(pady=20)
        
        # Create a frame for current velocity and adjustment buttons
        current_frame = ttk.Frame(velocity_frame)
        current_frame.pack(pady=10)
        
        # Add +/- 0.1 buttons next to current velocity display
        ttk.Button(current_frame, text="-0.1", bootstyle="warning", width=15, padding=10,
                  command=lambda: self.adjust_velocity(-0.1)).pack(side="left", padx=10)
        
        # Display current velocity value in the middle
        self.velocity_value_label = ttk.Label(current_frame, text=f"Current: {self.velocity:.2f}", font=("Arial", 36, "bold"))
        self.velocity_value_label.pack(side="left", padx=20)
        
        # Add +0.1 button on the right
        ttk.Button(current_frame, text="+0.1", bootstyle="warning", width=15, padding=10,
                  command=lambda: self.adjust_velocity(0.1)).pack(side="left", padx=10)
        
        # Add slider with large knob
        self.style = ttk.Style()
        self.style.configure('Large.Horizontal.TScale', 
                            sliderlength=3000,  # Very large slider knob
                            thickness=800,      # Very thick slider
                            borderwidth=30,     # Thick border
                            relief='solid', 
                            troughcolor='gray', 
                            gripcount=5)        # More grip lines
        
        # Create slider frame
        slider_frame = ttk.Frame(velocity_frame)
        slider_frame.pack(pady=20)
        
        # Add the slider
        self.speed_slider = ttk.Scale(slider_frame, from_=0.0, to=2.0, orient="horizontal", 
                                     length=900, style='Large.Horizontal.TScale',
                                     command=self.update_velocity_from_slider)
        self.speed_slider.set(self.velocity)
        self.speed_slider.pack(side="top", pady=10)

        jog_frame = ttk.Frame(self.root)
        jog_frame.pack(pady=50)

        self.btn_jog_minus = ttk.Button(jog_frame, text="◀ Jog -", bootstyle="danger", width=100, padding=50)
        self.btn_jog_minus.grid(row=0, column=0, padx=50)
        self.btn_jog_minus.bind("<ButtonPress>", lambda e: self.jog_negative())
        self.btn_jog_minus.bind("<ButtonRelease>", lambda e: self.stop_jog())

        self.btn_jog_plus = ttk.Button(jog_frame, text="Jog + ▶", bootstyle="success", width=100, padding=50)
        self.btn_jog_plus.grid(row=0, column=1, padx=50)
        self.btn_jog_plus.bind("<ButtonPress>", lambda e: self.jog_positive())
        self.btn_jog_plus.bind("<ButtonRelease>", lambda e: self.stop_jog())
        self.btn_home = ttk.Button(self.root, text="Home", bootstyle="primary", width=30, padding=30, command=self.execute_home)
        self.btn_home.pack(pady=20)  # ✅ เพิ่ม padding เพื่อให้ปุ่มแสดง

    
    def create_velocity_ui(self):
        ttk.Label(self.root, text="Set Joint Angles", font=("Arial", 36)).pack(pady=20)
        
        # ตรวจสอบว่ามีค่า joint_values อยู่แล้วหรือไม่ ถ้าไม่มีจึงสร้างขึ้นใหม่
        if not hasattr(self, 'joint_values'):
            self.joint_values = {joint: 0.0 for joint in self.joint_names}
            
        # คงค่า current_joint_index ไว้หากมีอยู่แล้ว หรือตั้งค่าเริ่มต้นถ้ายังไม่มีvelocity 
        # This frame will be positioned at the top-right corner
        self.special_frame = ttk.Frame(self.root)
        
        # Use place geometry manager to position the frame absolutely
        self.special_frame.place(relx=1.0, rely=0.0, anchor="ne", x=-20, y=20)
        
        # Toggle button for showing/hiding special actions
        self.show_special_actions = True
        self.toggle_actions_button = ttk.Button(
            self.special_frame, 
            text="Hide Special Actions", 
            bootstyle="secondary", 
            width=40, 
            padding=20, 
            command=self.toggle_special_actions
        )
        self.toggle_actions_button.pack(side="top", pady=5)
        
        # Special action buttons frame
        self.special_buttons_frame = ttk.Frame(self.special_frame)
        self.special_buttons_frame.pack(pady=5)
        
        # Add special action buttons
        self.btn_special_action = ttk.Button(
            self.special_buttons_frame, 
            text="Special Action", 
            bootstyle="warning", 
            width=40, 
            padding=20, 
            command=self.execute_special_action
        )
        self.btn_special_action.pack(side="top", pady=5)
        
        self.btn_special_action_2 = ttk.Button(
            self.special_buttons_frame, 
            text="Special Attack 2", 
            bootstyle="danger", 
            width=40, 
            padding=20, 
            command=self.execute_special_action_2
        )
        self.btn_special_action_2.pack(side="top", pady=5)
        
        self.btn_special_action_3 = ttk.Button(
            self.special_buttons_frame, 
            text="Special Action 3", 
            bootstyle="info", 
            width=40, 
            padding=20, 
            command=self.execute_special_action_3
        )
        self.btn_special_action_3.pack(side="top", pady=5)
        
        # Joint limits (degrees)
        self.joint_limits = {
            'joint1': (-180, 180),
            'joint2': (-90, 90),
            'joint3': (-90, 90),
            'joint4': (-180, 180),
            'joint5': (-180, 180),
            'joint6': (-180, 180)
        }
        
        # Initialize joint values dictionary to store entered values
        if not hasattr(self, 'joint_values'):
            self.joint_values = {joint: 0.0 for joint in self.joint_names}

        self.current_joint_index = 0
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(pady=20, fill="both", expand=True)
        
        # Create frame for the large input
        input_frame = ttk.Frame(main_frame)
        input_frame.pack(pady=20)
        
        # Current joint label
        self.current_joint_label = ttk.Label(input_frame, text=f"Joint: {self.joint_names[self.current_joint_index]}", 
                                            font=("Arial", 36, "bold"))
        self.current_joint_label.pack(pady=10)
        
        # Large input field for the current joint with much larger font
        self.current_joint_entry = ttk.Entry(input_frame, width=15, font=("Arial", 48))
        self.current_joint_entry.pack(pady=20)
        
        # Preset buttons frame
        preset_frame = ttk.Frame(input_frame)
        preset_frame.pack(pady=10)
        
        # Create preset buttons for the current joint
        current_joint = self.joint_names[self.current_joint_index]
        joint_min, joint_max = self.joint_limits[current_joint]
        step = (joint_max - joint_min) / 4
        
        for i in range(5):
            angle = int(joint_min + i * step)
            btn = ttk.Button(
                preset_frame, 
                text=f"{angle}°", 
                bootstyle="info", 
                width=40, 
                padding=20,
                command=lambda a=angle: self.set_preset_angle(a)
            )
            btn.pack(side="left", padx=15)
        
        # Add increment/decrement buttons for angle adjustment
        adjust_frame = ttk.Frame(input_frame)
        adjust_frame.pack(pady=10)
        
        # Add +/- 1 degree buttons
        ttk.Button(adjust_frame, text="-1°", bootstyle="secondary", width=20, padding=20,
                  command=lambda: self.adjust_angle(-1)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+1°", bootstyle="secondary", width=20, padding=20,
                  command=lambda: self.adjust_angle(1)).pack(side="left", padx=10)
        
        # Add +/- 5 degree buttons
        ttk.Button(adjust_frame, text="-5°", bootstyle="info", width=20, padding=20,
                  command=lambda: self.adjust_angle(-5)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+5°", bootstyle="info", width=20, padding=20,
                  command=lambda: self.adjust_angle(5)).pack(side="left", padx=10)
        
        # Add +/- 10 degree buttons
        ttk.Button(adjust_frame, text="-10°", bootstyle="primary", width=20, padding=20,
                  command=lambda: self.adjust_angle(-10)).pack(side="left", padx=10)
        ttk.Button(adjust_frame, text="+10°", bootstyle="primary", width=20, padding=20,
                  command=lambda: self.adjust_angle(10)).pack(side="left", padx=10)
        
        # Navigation and action buttons in one frame
        nav_action_frame = ttk.Frame(main_frame)
        nav_action_frame.pack(pady=20)
        
        # Navigation buttons
        nav_frame = ttk.Frame(nav_action_frame)
        nav_frame.pack(side="left", padx=20)
        
        self.prev_button = ttk.Button(nav_frame, text="◀ Previous", bootstyle="secondary", 
                                     width=40, padding=30, command=self.previous_joint)
        self.prev_button.pack(side="left", padx=15)
        
        self.next_button = ttk.Button(nav_frame, text="Next ▶", bootstyle="primary", 
                                     width=40, padding=30, command=self.next_joint)
        self.next_button.pack(side="left", padx=15)
        
        # สร้าง Frame สำหรับแสดงข้อมูล MQTT
        self.mqtt_data_frame = ttk.Frame(self.root)
        self.mqtt_data_frame.pack(pady=10)

        # สร้าง Label สำหรับแสดงข้อความที่ได้รับจาก MQTT
        self.mqtt_data_label = ttk.Label(self.mqtt_data_frame, text="MQTT Data: Waiting for data...",
                                        font=("Arial", 20), bootstyle="info")
        self.mqtt_data_label.pack()

        
        # Action buttons next to navigation buttons
        action_frame = ttk.Frame(nav_action_frame)
        action_frame.pack(side="left", padx=20)
        
        # Main action buttons
        self.btn_send_angles = ttk.Button(
            action_frame, 
            text="Send Goal", 
            bootstyle="secondary", 
            width=40, 
            padding=30, 
            command=self.send_joint_angles
        )
        self.btn_send_angles.pack(side="left", padx=15)
        
        self.btn_home = ttk.Button(
            action_frame, 
            text="Home", 
            bootstyle="primary", 
            width=40, 
            padding=30, 
            command=self.execute_home
        )
        self.btn_home.pack(side="left", padx=15)
        
        ######################################################
        # MQTT Configuration และ Toggle Frame
        mqtt_config_frame = ttk.Frame(self.root)
        mqtt_config_frame.pack(side="bottom", pady=10)

        # บรรทัดที่ 1: Broker, Port, Test Button และ Status Result
        broker_config_frame = ttk.Frame(mqtt_config_frame)
        broker_config_frame.pack(pady=5)

        # Label สำหรับ Broker
        ttk.Label(broker_config_frame, text="MQTT Broker:", font=("Arial", 16)).pack(side="left", padx=5)

        # Entry สำหรับ MQTT Broker
        self.broker_entry = ttk.Entry(broker_config_frame, width=25, font=("Arial", 14))
        self.broker_entry.pack(side="left", padx=5)
        self.broker_entry.insert(0, "test.mosquitto.org")

        # Label สำหรับ Port
        ttk.Label(broker_config_frame, text="Port:", font=("Arial", 16)).pack(side="left", padx=(20, 5))

        # Entry สำหรับ Port
        self.port_entry = ttk.Entry(broker_config_frame, width=8, font=("Arial", 14))
        self.port_entry.pack(side="left", padx=5)
        self.port_entry.insert(0, "1883")

        # ปุ่มทดสอบการเชื่อมต่อ
        self.test_connection_button = ttk.Button(
            broker_config_frame,
            text="Test Connection",
            bootstyle="info",
            width=15,
            padding=10,
            command=self.test_mqtt_connection
        )
        self.test_connection_button.pack(side="left", padx=10)

        # ✅ แสดงสถานะการเชื่อมต่อ ข้างๆ ปุ่ม Test
        self.connection_status_label = ttk.Label(
            broker_config_frame, 
            text="Status: Not Connected", 
            font=("Arial", 12),
            bootstyle="secondary",
            width=25  # กำหนดความกว้างคงที่
        )
        self.connection_status_label.pack(side="left", padx=10)

        # บรรทัดที่ 2: MQTT Toggle
        mqtt_toggle_frame = ttk.Frame(mqtt_config_frame)
        mqtt_toggle_frame.pack(pady=5)

        self.mqtt_toggle_label = ttk.Label(mqtt_toggle_frame, text="MQTT: OFF", font=("Arial", 24))
        self.mqtt_toggle_label.pack(side="left", padx=20)

        self.mqtt_toggle_button = ttk.Button(
            mqtt_toggle_frame, 
            text="Enable MQTT", 
            bootstyle="success", 
            width=20, 
            padding=20, 
            command=self.toggle_mqtt
        )
        self.mqtt_toggle_button.pack(side="left", padx=10)
        ###################################################

        self.cartesian_data_frame = ttk.Frame(self.root)

        # กำหนดตำแหน่งให้อยู่ขวาล่างแต่ไม่ตกขอบ
        self.cartesian_data_frame.place(relx=0.98, rely=0.98, anchor="se")

        # Title Label
        ttk.Label(self.cartesian_data_frame, text="Cartesian Data", font=("Arial", 18, "bold")).pack(anchor="w")

        # Dictionary เก็บ Label สำหรับ X, Y, Z, Roll, Pitch, Yaw
        self.cartesian_labels = {}
        cartesian_params = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        for param in cartesian_params:
            frame = ttk.Frame(self.cartesian_data_frame)
            frame.pack(anchor="w", pady=2)

            ttk.Label(frame, text=f"{param}:", font=("Arial", 16)).pack(side="left")

            label = ttk.Label(frame, text="0.0 mm" if param in ["X", "Y", "Z"] else "0.0°",
                            font=("Arial", 16, "bold"))
            label.pack(side="left", padx=10)

            self.cartesian_labels[param] = label

        # Display saved values in a more compact way
        self.values_display_frame = ttk.Frame(main_frame)
        self.values_display_frame.pack(pady=10, fill="x")
        
        # Create a label for each joint value
        self.joint_value_labels = {}
        joint_values_grid = ttk.Frame(self.values_display_frame)
        joint_values_grid.pack(padx=10, pady=5)
        
        # Create a grid of labels for joint values (3 columns)
        for i, joint in enumerate(self.joint_names):
            row = i // 3
            col = i % 3
            
            # Create a frame for each joint value pair
            joint_frame = ttk.Frame(joint_values_grid)
            joint_frame.grid(row=row, column=col, padx=10, pady=5)
            # Add joint name and value
            ttk.Label(joint_frame, text=f"{joint}:", 
                     font=("Arial", 16, "bold")).pack(side="left")
            
            value_label = ttk.Label(joint_frame, text="0.0°", 
                                   font=("Arial", 16))
            value_label.pack(side="left", padx=5)
            
            self.joint_value_labels[joint] = value_label
        
        self.update_values_display()
    def update_cartesian_values(self, x, y, z, roll, pitch, yaw):
        """อัปเดตค่า Cartesian Control UI"""
        
        self.ros_node.get_logger().info(f"🔄 Updating Cartesian Values: X={x}mm, Y={y}mm, Z={z}mm, Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")

        # ตรวจสอบก่อนว่า entry widgets มีอยู่จริง
        if self.cartesian_entries:
            if "x" in self.cartesian_entries and self.cartesian_entries["x"].winfo_exists():
                self.cartesian_entries['x'].delete(0, 'end')
                self.cartesian_entries['x'].insert(0, str(x))

            if "y" in self.cartesian_entries and self.cartesian_entries["y"].winfo_exists():
                self.cartesian_entries['y'].delete(0, 'end')
                self.cartesian_entries['y'].insert(0, str(y))

            if "z" in self.cartesian_entries and self.cartesian_entries["z"].winfo_exists():
                self.cartesian_entries['z'].delete(0, 'end')
                self.cartesian_entries['z'].insert(0, str(z))

            if "roll" in self.cartesian_entries and self.cartesian_entries["roll"].winfo_exists():
                self.cartesian_entries['roll'].delete(0, 'end')
                self.cartesian_entries['roll'].insert(0, str(roll))

            if "pitch" in self.cartesian_entries and self.cartesian_entries["pitch"].winfo_exists():
                self.cartesian_entries['pitch'].delete(0, 'end')
                self.cartesian_entries['pitch'].insert(0, str(pitch))

            if "yaw" in self.cartesian_entries and self.cartesian_entries["yaw"].winfo_exists():
                self.cartesian_entries['yaw'].delete(0, 'end')
                self.cartesian_entries['yaw'].insert(0, str(yaw))

        # ✅ อัปเดต Label ที่มุมขวาล่าง
        if self.cartesian_labels:
            if "X" in self.cartesian_labels:
                self.cartesian_labels["X"].config(text=f"{x:.2f} mm")

            if "Y" in self.cartesian_labels:
                self.cartesian_labels["Y"].config(text=f"{y:.2f} mm")

            if "Z" in self.cartesian_labels:
                self.cartesian_labels["Z"].config(text=f"{z:.2f} mm")

            if "Roll" in self.cartesian_labels:
                self.cartesian_labels["Roll"].config(text=f"{roll:.2f}°")

            if "Pitch" in self.cartesian_labels:
                self.cartesian_labels["Pitch"].config(text=f"{pitch:.2f}°")

            if "Yaw" in self.cartesian_labels:
                self.cartesian_labels["Yaw"].config(text=f"{yaw:.2f}°")

        # ✅ Refresh UI
        self.root.update()


    def clear_all_entries(self):
        """ล้างค่าช่องกรอกทั้งหมด และอัปเดต UI"""
        self.ros_node.get_logger().info("🔄 Clearing all joint and Cartesian entries")

        # ✅ ล้างค่าช่องกรอกของ Joint
        if hasattr(self, 'current_joint_entry'):
            self.current_joint_entry.delete(0, 'end')

        for joint in self.joint_names:
            self.joint_values[joint] = 0.0  # รีเซ็ตค่าเป็น 0
            if joint in self.joint_value_labels:
                self.joint_value_labels[joint].config(text="0.0°")  # ✅ อัปเดต Label ให้เปลี่ยนด้วย

        # ✅ อัปเดตชื่อ Joint ปัจจุบัน
        self.current_joint_label.config(text=f"Joint: {self.joint_names[self.current_joint_index]}")  

        # ✅ ล้างค่าช่องกรอกของ Cartesian Control
        for param in ["x", "y", "z", "roll", "pitch", "yaw"]:
            if param in self.cartesian_entries:
                self.cartesian_entries[param].delete(0, 'end')
                self.cartesian_entries[param].insert(0, "0.0")
                self.cartesian_values[param] = 0.0  # รีเซ็ตค่าเป็น 0

            if param.capitalize() in self.cartesian_labels:
                self.cartesian_labels[param.capitalize()].config(text="0.0 mm" if param in ["x", "y", "z"] else "0.0°")  

        # ✅ รีเฟรช UI
        self.root.update_idletasks()
    def test_mqtt_connection(self):
        """ทดสอบการเชื่อมต่อ MQTT Broker"""
        broker, port = self.get_mqtt_config()
        
        self.connection_status_label.config(text="Testing...", bootstyle="warning")
        self.root.update()
        
        try:
            # ✅ ใช้ MQTT Client v2 พร้อม fallback
            try:
                test_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
                self.ros_node.get_logger().info("Testing with MQTT Client v2")
            except (AttributeError, NameError):
                # Fallback สำหรับ MQTT version เก่า
                test_client = mqtt.Client()
                self.ros_node.get_logger().info("Testing with MQTT Client v1 (fallback)")
                
            test_client.on_connect = self.on_test_connect
            test_client.on_disconnect = self.on_test_disconnect
            
            test_client.connect(broker, port, 10)
            test_client.loop_start()
            
            # รอผลลัพธ์ 3 วินาที
            self.root.after(3000, lambda: self.finish_connection_test(test_client))
            
            self.ros_node.get_logger().info(f"Testing connection to {broker}:{port}")
            
        except Exception as e:
            self.connection_status_label.config(
                text=f"Failed: {str(e)[:15]}...", 
                bootstyle="danger"
            )
            self.ros_node.get_logger().error(f"MQTT Test Connection Failed: {e}")

    #def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
    def on_mqtt_connect(self, client, userdata, flags, rc, *args):
        """Callback เมื่อเชื่อมต่อ MQTT สำเร็จ (MQTT v2 compatible)"""
        if rc == 0:
            self.connection_status_label.config(
                text="✅ MQTT Connected!", 
                bootstyle="success"
            )
            self.ros_node.get_logger().info("MQTT Connected successfully")
        else:
            self.connection_status_label.config(
                text=f"❌ MQTT Failed (Code: {rc})", 
                bootstyle="danger"
            )
            self.ros_node.get_logger().error(f"MQTT Connection failed with code: {rc}")

    #def on_mqtt_disconnect(self, client, userdata, rc, properties=None):
    def on_mqtt_disconnect(self, client, userdata, rc, *args):
        """Callback เมื่อ MQTT disconnect (MQTT v2 compatible)"""
        self.connection_status_label.config(
            text="Status: Disconnected", 
            bootstyle="secondary"
        )
        self.ros_node.get_logger().info("MQTT Disconnected")


    def start_mqtt_subscription(self):
        """เริ่มต้นรับค่าจาก MQTT โดยใช้ค่าจาก UI"""
        broker, port = self.get_mqtt_config()
        
        # ✅ ใช้ MQTT Client v2 พร้อม fallback
        try:
            self.mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
            self.ros_node.get_logger().info("Using MQTT Client v2")
        except (AttributeError, NameError):
            # Fallback สำหรับ MQTT version เก่า
            self.mqtt_client = mqtt.Client()
            self.ros_node.get_logger().info("Using MQTT Client v1 (fallback)")
            
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        try:
            self.mqtt_client.connect(broker, port, 60)
            self.mqtt_client.subscribe(f"{self.robot_name}/pose")
            self.mqtt_client.subscribe(f"{self.robot_name}/angles")
            self.mqtt_client.loop_start()
            
            self.ros_node.get_logger().info(f"Connecting to MQTT Broker: {broker}:{port}")
            
        except Exception as e:
            self.connection_status_label.config(
                text=f"❌ Failed: {str(e)[:15]}...", 
                bootstyle="danger"
            )
            self.ros_node.get_logger().error(f"MQTT Connection Failed: {e}")

    # แก้ไข stop_mqtt_subscription() ให้อัปเดต Status:
    def stop_mqtt_subscription(self):
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception as e:
                self.ros_node.get_logger().warning(f"Error stopping MQTT: {e}")
            finally:
                if hasattr(self, 'connection_status_label'):
                    self.connection_status_label.config(text="Status: Disconnected", bootstyle="secondary")
                self.ros_node.get_logger().info("MQTT Disconnected")

    # แก้ไข toggle_mqtt() ให้ใช้ค่าจาก UI:
    def toggle_mqtt(self):
        """เปิด-ปิด MQTT subscription โดยใช้ค่าจาก UI"""
        self.mqtt_toggle_state = not self.mqtt_toggle_state

        if self.mqtt_toggle_state:
            # ✅ ตรวจสอบค่า broker และ port ก่อนเชื่อมต่อ
            broker, port = self.get_mqtt_config()
            
            self.mqtt_toggle_label.config(text="MQTT: ON")
            self.mqtt_toggle_button.config(text="Disable MQTT", bootstyle="danger")
            self.start_mqtt_subscription()
            # ✅ เพิ่มข้อมูล broker ใน log
            self.ros_node.get_logger().info(f"MQTT Subscription: ENABLED (Broker: {broker}:{port})")

        else:
            self.mqtt_toggle_label.config(text="MQTT: OFF")
            self.mqtt_toggle_button.config(text="Enable MQTT", bootstyle="success")
            self.stop_mqtt_subscription()
            self.ros_node.get_logger().info("MQTT Subscription: DISABLED")
    def on_test_connect(self, client, userdata, flags, rc, *args):
        """Callback สำหรับการทดสอบการเชื่อมต่อ (MQTT v2 compatible)"""
        if rc == 0:
            self.connection_status_label.config(text="✅ Connection Successful!", bootstyle="success")
            self.ros_node.get_logger().info("MQTT Test Connection: SUCCESS")
        else:
            self.connection_status_label.config(text=f"❌ Connection Failed (Code: {rc})", bootstyle="danger")
            self.ros_node.get_logger().error(f"MQTT Test Connection Failed with code: {rc}")
    def on_test_disconnect(self, client, userdata, rc, *args):
        """Callback เมื่อการทดสอบเสร็จสิ้น (MQTT v2 compatible)"""
        self.ros_node.get_logger().info("MQTT Test Connection: Disconnected")


    def on_mqtt_message(self, client, userdata, msg, *args):
        """ประมวลผลข้อความที่ได้รับจาก MQTT (MQTT v2 compatible)"""
        try:
            data = msg.payload.decode()
            topic = msg.topic

            self.ros_node.get_logger().info(f"[MQTT] Received: {topic} -> {data}")
            
            # อัปเดต MQTT Data Label
            if hasattr(self, 'mqtt_data_label'):
                short_data = data[:30] + "..." if len(data) > 30 else data
                self.mqtt_data_label.config(text=f"MQTT: {topic.split('/')[-1]} -> {short_data}")

            if topic.endswith("/pose"):
                self.process_pose_data(data)
            elif topic.endswith("/angles"):
                self.process_angles_data(data)

        except Exception as e:
            self.ros_node.get_logger().error(f"[MQTT] Message processing failed: {e}")
    def process_pose_data(self, data):
        """ประมวลผลข้อมูล Pose จาก MQTT"""
        try:
            x, y, z, roll, pitch, yaw = map(float, data.split(","))
            self.ros_node.get_logger().info(f"[POSE] X={x}, Y={y}, Z={z}, R={roll}, P={pitch}, Y={yaw}")
            
            if self.kinematics is None:
                self.ros_node.get_logger().error("[ERROR] Kinematics not initialized")
                return

            # Convert to meters and radians
            x_m, y_m, z_m = x / 1000, y / 1000, z / 1000
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(yaw)

            # Compute inverse kinematics
            joint_angles = self.kinematics.compute_ink([x_m, y_m, z_m], [roll_rad, pitch_rad, yaw_rad])
            
            if joint_angles:
                joint_angles_deg = [math.degrees(angle) for angle in joint_angles]
                self.update_joint_angles(joint_angles_deg)
                self.send_joint_angles()
            
            # Update Cartesian display
            self.update_cartesian_values(x, y, z, roll, pitch, yaw)
            
        except Exception as e:
            self.ros_node.get_logger().error(f"[ERROR] Pose processing failed: {e}")

    def process_angles_data(self, data):
        """ประมวลผลข้อมูล Joint Angles จาก MQTT"""
        try:
            joint_angles_deg = list(map(float, data.split(",")))
            self.ros_node.get_logger().info(f"[ANGLES] Received: {joint_angles_deg}")
            
            if self.kinematics:
                # Convert to radians for forward kinematics
                joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
                
                # Compute forward kinematics
                try:
                    x, y, z, roll, pitch, yaw = self.kinematics.compute_fk(joint_angles_rad)
                    x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000
                    roll_deg = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg = math.degrees(yaw)
                    
                    self.update_cartesian_values(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)
                except Exception as e:
                    self.ros_node.get_logger().error(f"[ERROR] Forward kinematics failed: {e}")

            self.update_joint_angles(joint_angles_deg, update_ui=True)
            self.send_joint_angles()
            
        except Exception as e:
            self.ros_node.get_logger().error(f"[ERROR] Angles processing failed: {e}")
    def update_joint_angles(self, angles, update_ui=True):
        """อัปเดตค่า Joint Angle UI และส่งออกไปทันที"""
        self.ros_node.get_logger().info(f"📥 Received Raw Joint Angles: {angles}")

        # ✅ ตรวจสอบว่าค่าที่ได้รับมีจำนวนตรงกับ joint_names
        if len(angles) != len(self.joint_names):
            self.ros_node.get_logger().error("❌ Error: จำนวน joint angles ไม่ตรงกับที่กำหนด")
            return

        # ✅ อัปเดตค่าในตัวแปร joint_values
        for i, joint in enumerate(self.joint_names):
            self.joint_values[joint] = angles[i]
            self.ros_node.get_logger().info(f"Updated {joint}: {self.joint_values[joint]}°")

        # ✅ อัปเดต UI
        if update_ui:
            for i, joint in enumerate(self.joint_names):
                if joint in self.joint_value_labels:
                    self.joint_value_labels[joint].config(text=f"{angles[i]:.2f}°")

            # ✅ อัปเดตช่องกรอกค่าของ joint ปัจจุบัน
            if self.current_joint in self.joint_names and hasattr(self, 'current_joint_entry'):
                self.current_joint_entry.delete(0, 'end')
                self.current_joint_entry.insert(0, str(self.joint_values[self.current_joint]))

            # ✅ Refresh UI
            self.root.update_idletasks()







    def toggle_all_buttons(self, state):
            """เปิดหรือปิดการใช้งานปุ่มทั้งหมด และเปลี่ยนสีปุ่มให้เป็นเทาเมื่อปิด"""
            is_disabled = (state == "disable")

            # ✅ ใช้ recursive function เพื่อตรวจสอบวิดเจ็ตทั้งหมด
            def disable_recursively(widget):
                """ปิดทุกวิดเจ็ตใน UI รวมถึงที่อยู่ในเฟรมซ้อนกัน"""
                if isinstance(widget, ttk.Button) and widget != self.mqtt_toggle_button:
                    widget["state"] = "disabled" if is_disabled else "normal"
                    widget.configure(bootstyle="secondary" if is_disabled else "primary")  

                elif isinstance(widget, (ttk.Entry, ttk.Combobox, ttk.Scale)):
                    widget["state"] = "disabled" if is_disabled else "normal"

                # ✅ ค้นหาและปิด widget ที่อยู่ใน frame ซ้อนกัน
                if isinstance(widget, ttk.Frame):
                    for child in widget.winfo_children():
                        disable_recursively(child)

            # ✅ ปิดทุก widget ที่อยู่ใน root และเฟรมทั้งหมด
            for widget in self.root.winfo_children():
                disable_recursively(widget)






    def execute_special_action(self):
        """ Execute a predefined sequence of joint positions """
        sequence = [
            [90.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [90.0, -45.0, 20.0, 90.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, -45.0, 20.0, 90.0, 0.0, 0.0]
        ]

        self.execute_sequence(sequence)
    
    def execute_special_action_2(self):
        """ Execute another predefined sequence of joint positions """
        sequence = [
            [-100.0, -45.0, 0.0, 0.0, 0.0, 0.0],
            [-100.0, 0.0, -70.0, 0.0, 0.0, 0.0],
            [0.0, -45.0, 0.0, 0.0, 0.0, 0.0],
            [-90.0, -45.0, 0.0, 0.0, 0.0, 0.0]
        ]

        self.execute_sequence(sequence)
    def execute_special_action_3(self):
        """ Execute a randomized sequence of joint positions within 60% of their limits """
        sequence = []
        for _ in range(10):
            sequence.append([
                random.uniform(self.joint_limits[joint][0] * 0.6, self.joint_limits[joint][1] * 0.6)
                for joint in self.joint_names
            ])
        self.execute_sequence(sequence)
   
    def execute_home(self):
        """Move the robot to home position and publish 'm' to /control_mode/state"""
        self.ros_node.get_logger().info("📢 Sending 'm' to switch to Home Position Mode")
        
        # ✅ ส่ง 'm' ไปที่ /control_mode/state
        msg = String()
        msg.data = 'm'
        self.control_mode_publisher.publish(msg)

        # ✅ เรียกฟังก์ชันให้หุ่นยนต์กลับ Home
        # sequence = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        # self.execute_sequence(sequence)
    def execute_sequence(self, sequence):
         if not self.client.wait_for_service(timeout_sec=1.0):
             self.ros_node.get_logger().error("Service /set_position is not available.")
             return

         for angles in sequence:
             request = SetPosition.Request()
             request.target_positions = list(map(float, angles))  # Ensure all values are float
             future = self.client.call_async(request)
             future.add_done_callback(self.handle_service_response)
             self.ros_node.get_logger().info(f"Sent Sequence: {angles}")
             rclpy.spin_until_future_complete(self.ros_node, future)
            
             # Add delay between actions
             self.root.after(2000)
 
    def set_mode(self, mode):
        if mode == self.control_mode:
            return
            
        # บันทึกค่า joint_values ในโหมดปัจจุบันก่อนเปลี่ยนโหมด
        if hasattr(self, 'current_joint_entry') and self.control_mode == "velocity":
            self.save_current_joint_value()
                
        self.control_mode = mode
        self.publish_control_mode_state()
        self.create_ui()
        
        # พิมพ์ค่า joint_values หลังจากเปลี่ยนโหมดเพื่อตรวจสอบว่าค่ายังคงอยู่
        if hasattr(self, 'joint_values'):
            self.ros_node.get_logger().info(f"Joint values after mode change: {self.joint_values}")

        
    def toggle_mode(self):
        # For backward compatibility
        if self.control_mode == "jog":
            self.set_mode("velocity")
        elif self.control_mode == "velocity":
            self.set_mode("jog")
        else:
            self.set_mode("jog")
    def publish_control_mode_state(self):
        """ Publish the current control mode to /control_mode/state """
        msg = String()
        msg.data = self.control_mode
        self.control_mode_publisher.publish(msg)
        self.ros_node.get_logger().info(f"Published control mode: {self.control_mode}")
        
    def create_cartesian_ui(self):
        """Create UI for Cartesian coordinate control"""

        # ตรวจสอบว่ามีค่า joint_values อยู่แล้วหรือไม่ ถ้าไม่มีจึงสร้างขึ้นใหม่
        if not hasattr(self, 'joint_values'):
            self.joint_values = {joint: 0.0 for joint in self.joint_names}
        
        # Create a simple title
        ttk.Label(self.root, text="Cartesian Control", font=("Arial", 36)).pack(pady=20)
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(pady=20, fill="both", expand=True)
        
        # Create left frame for coordinate inputs
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side="left", padx=20, fill="both", expand=True)
        
        # Create right frame for current joint values display
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side="right", padx=20, fill="both", expand=True)
        
        # Create coordinate input fields
        coord_frame = ttk.Frame(left_frame)
        coord_frame.pack(pady=20, fill="x")
        
        # Dictionary to store entry widgets
        self.cartesian_entries = {}
        
        # Create input fields for X, Y, Z, Roll, Pitch, Yaw
        cartesian_params = [
            ('x', 'X (mm)'),
            ('y', 'Y (mm)'),
            ('z', 'Z (mm)'),
            ('roll', 'Roll (deg)'),
            ('pitch', 'Pitch (deg)'),
            ('yaw', 'Yaw (deg)')
        ]
        
        for i, (param, label) in enumerate(cartesian_params):
            param_frame = ttk.Frame(coord_frame)
            param_frame.pack(pady=10, fill="x")
            
            ttk.Label(param_frame, text=label, font=("Arial", 24)).pack(side="left", padx=10)
            
            entry = ttk.Entry(param_frame, width=10, font=("Arial", 24))
            entry.pack(side="left", padx=10)
            entry.insert(0, str(self.cartesian_values[param]))
            
            # Add buttons for adjusting values
            btn_frame = ttk.Frame(param_frame)
            btn_frame.pack(side="left", padx=10)
            
            # Determine step size based on parameter
            step = 10.0 if param in ['x', 'y', 'z'] else 5.0
            
            ttk.Button(btn_frame, text=f"-{step}", bootstyle="warning", width=8, padding=10,
                      command=lambda p=param, s=-step: self.adjust_cartesian(p, s)).pack(side="left", padx=5)
            
            ttk.Button(btn_frame, text=f"+{step}", bootstyle="success", width=8, padding=10,
                      command=lambda p=param, s=step: self.adjust_cartesian(p, s)).pack(side="left", padx=5)
            
            self.cartesian_entries[param] = entry
        
        # Add buttons for sending commands
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(pady=20)
        
        ttk.Button(btn_frame, text="Send to Robot", bootstyle="success", width=30, padding=30,
                  command=self.send_cartesian_command).pack(side="left", padx=20)
        
        ttk.Button(btn_frame, text="Reset Values", bootstyle="warning", width=30, padding=30,
                  command=self.reset_cartesian_values).pack(side="left", padx=20)
        
        # Display current joint values in right frame
        ttk.Label(right_frame, text="Current Joint Values", font=("Arial", 24, "bold")).pack(pady=10)
        self.btn_home = ttk.Button(btn_frame, text="Home", bootstyle="primary", width=30, padding=30, command=self.execute_home)
        self.btn_home.pack(side="left", padx=20)

        
        self.joint_values_frame = ttk.Frame(right_frame)
        self.joint_values_frame.pack(pady=10, fill="x")
        
        # Create labels for joint values
        self.joint_value_labels = {}
        for i, joint in enumerate(self.joint_names):
            joint_frame = ttk.Frame(self.joint_values_frame)
            joint_frame.pack(pady=5, fill="x")
            
            ttk.Label(joint_frame, text=f"{joint}:", font=("Arial", 18, "bold")).pack(side="left", padx=10)
            
            value_label = ttk.Label(joint_frame, text="0.0°", font=("Arial", 18))
            value_label.pack(side="left", padx=5)
            
            self.joint_value_labels[joint] = value_label
 
    def send_velocity(self, vel):
        joint_index = self.joint_names.index(self.joint_selector.get())
        velocities = [0.0] * len(self.joint_names)
        velocities[joint_index] = vel
        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)
        self.ros_node.get_logger().info(f"Publishing: {velocities}")
    
    def jog_positive(self):
        velocity = self.speed_slider.get()
        self.send_velocity(velocity)
    
    def jog_negative(self):
        velocity = -self.speed_slider.get()
        self.send_velocity(velocity)
    
    def stop_jog(self):
        self.send_velocity(0.0)
    
    def set_preset_angle(self, angle):
        """Set a preset angle value in the current entry field"""
        self.current_joint_entry.delete(0, 'end')
        self.current_joint_entry.insert(0, str(angle))
        self.save_current_joint_value()
    
    def next_joint(self):
        """Move to the next joint"""
        self.save_current_joint_value()
        self.current_joint_index = (self.current_joint_index + 1) % len(self.joint_names)
        self.update_current_joint_display()
    
    def previous_joint(self):
        """Move to the previous joint"""
        self.save_current_joint_value()
        self.current_joint_index = (self.current_joint_index - 1) % len(self.joint_names)
        self.update_current_joint_display()
    
    def save_current_joint_value(self):
        """Save the current joint value"""
        try:
            value = float(self.current_joint_entry.get())
            self.joint_values[self.joint_names[self.current_joint_index]] = value
            self.update_values_display()
        except ValueError:
            # If the entry is empty or invalid, don't update
            pass
    
    def update_current_joint_display(self):
        """Update the display for the current joint"""
        current_joint = self.joint_names[self.current_joint_index]
        self.current_joint_label.config(text=f"Joint: {current_joint}")
        
        # Update entry with saved value
        self.current_joint_entry.delete(0, 'end')
        self.current_joint_entry.insert(0, str(self.joint_values[current_joint]))
        
        # Update preset buttons
        self.update_preset_buttons()
        self.root.update_idletasks()
    
    def update_preset_buttons(self):
        """Update preset buttons for the current joint"""
        # Clear existing buttons in preset frame
        for widget in self.root.nametowidget(self.current_joint_entry.winfo_parent()).winfo_children()[2].winfo_children():
            widget.destroy()
        
        # Get current joint
        current_joint = self.joint_names[self.current_joint_index]
        joint_min, joint_max = self.joint_limits[current_joint]
        
        # Calculate step size for 5 buttons
        step = (joint_max - joint_min) / 4
        
        # Create preset buttons
        preset_frame = self.root.nametowidget(self.current_joint_entry.winfo_parent()).winfo_children()[2]
        for i in range(5):
            angle = int(joint_min + i * step)
            btn = ttk.Button(
                preset_frame, 
                text=f"{angle}°", 
                bootstyle="info", 
                width=40, 
                padding=20,
                command=lambda a=angle: self.set_preset_angle(a)
            )
            btn.pack(side="left", padx=15)
    
    def update_values_display(self):
        """Update the display of saved joint values"""
        for joint, value in self.joint_values.items():
            self.joint_value_labels[joint].config(text=f"{value}°")
    
    def adjust_angle(self, amount):
        """Adjust the current angle by the specified amount"""
        try:
            current_value = float(self.current_joint_entry.get())
            new_value = current_value + amount
            self.current_joint_entry.delete(0, 'end')
            self.current_joint_entry.insert(0, str(new_value))
            self.save_current_joint_value()
        except ValueError:
            # If the entry is empty or invalid, start from 0
            self.current_joint_entry.delete(0, 'end')
            self.current_joint_entry.insert(0, str(amount))
            self.save_current_joint_value()
    
    def toggle_special_actions(self):
        """Toggle visibility of special action buttons"""
        self.show_special_actions = not self.show_special_actions
        
        if self.show_special_actions:
            self.special_buttons_frame.pack(pady=5)
            self.toggle_actions_button.config(text="Hide Special Actions")
        else:
            self.special_buttons_frame.pack_forget()
            self.toggle_actions_button.config(text="Show Special Actions")
    
    def send_joint_angles(self):
        if self.control_mode == "velocity" and hasattr(self, 'current_joint_entry'):
            self.save_current_joint_value()

        # ตรวจสอบค่าก่อนสร้าง angles array
        for joint in self.joint_names:
            self.ros_node.get_logger().info(f"🔍 Before Copy: {joint} = {self.joint_values[joint]}")

        self.ros_node.get_logger().info(f"🔍 Full Joint Values before copying: {self.joint_values}")

        angles = [self.joint_values[joint] for joint in self.joint_names].copy()

        # ตรวจสอบค่าหลัง Copy
        for i, joint in enumerate(self.joint_names):
            self.ros_node.get_logger().info(f"🔍 After Copy: {joint} = {angles[i]}")

        self.ros_node.get_logger().info(f"🔍 Full Joint Values before sending: {angles}")

        angles[2] = -angles[2]
        angles[3] = -angles[3]
        angles[4] = angles[4] - angles[3]

        self.ros_node.get_logger().info(f"🔍 Modified Joint Angles before sending: {angles}")

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().error("Service /set_position is not available.")
            return

        request = SetPosition.Request()
        request.target_positions = angles

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)

        self.root.update()



    def handle_service_response(self, future):
        try:
            response = future.result()
            self.ros_node.get_logger().info(f"Service Response: {response}")
        except Exception as e:
            self.ros_node.get_logger().error(f"Service call failed: {str(e)}")
            
    def adjust_cartesian(self, param, step):
        """Adjust cartesian parameter by the specified step"""
        try:
            current_value = float(self.cartesian_entries[param].get())
            new_value = current_value + step
            
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(new_value))
            
            self.cartesian_values[param] = new_value
        except ValueError:
            # If the entry is empty or invalid, reset to default
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(self.cartesian_values[param]))
    
    def reset_cartesian_values(self):
        """Reset cartesian values to defaults"""
        default_values = {
            'x': 100.0,
            'y': 0.0,
            'z': 100.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        for param, value in default_values.items():
            self.cartesian_entries[param].delete(0, 'end')
            self.cartesian_entries[param].insert(0, str(value))
            self.cartesian_values[param] = value
    
    def send_cartesian_command(self):
        """Convert cartesian coordinates to joint angles using the SolveIK service and send to robot"""
        # Get values from entry fields
        try:
            x = float(self.cartesian_entries['x'].get())
            y = float(self.cartesian_entries['y'].get())
            z = float(self.cartesian_entries['z'].get())
            roll = float(self.cartesian_entries['roll'].get())
            pitch = float(self.cartesian_entries['pitch'].get())
            yaw = float(self.cartesian_entries['yaw'].get())
            
            # Update stored values
            self.cartesian_values['x'] = x
            self.cartesian_values['y'] = y
            self.cartesian_values['z'] = z
            self.cartesian_values['roll'] = roll
            self.cartesian_values['pitch'] = pitch
            self.cartesian_values['yaw'] = yaw
            
            self.ros_node.get_logger().info(f"Calling SolveIK service for: x={x}, y={y}, z={z}, rx={roll}, ry={pitch}, rz={yaw}")
            
            # Check if service is available
            if not self.solve_ik_client.wait_for_service(timeout_sec=1.0):
                self.ros_node.get_logger().error("Service /solve_ik is not available.")
                return
            
            # Create and send request to SolveIK service
            request = SolveIK.Request()
            request.x = x
            request.y = y
            request.z = z
            request.rx = roll
            request.ry = pitch
            request.rz = yaw
            
            future = self.solve_ik_client.call_async(request)
            
            # Add callback to handle the response
            future.add_done_callback(self.handle_solve_ik_response)
            
            # Wait for the response
            rclpy.spin_until_future_complete(self.ros_node, future)
                
        except ValueError as e:
            self.ros_node.get_logger().error(f"Invalid input: {e}")
        except Exception as e:
            self.ros_node.get_logger().error(f"Error sending cartesian command: {str(e)}")
            import traceback
            self.ros_node.get_logger().error(traceback.format_exc())
        
    def handle_solve_ik_response(self, future):
        """Handle the response from the SolveIK service"""
        try:
            response = future.result()
            self.ros_node.get_logger().info(f"SolveIK Response: success={response.success}, message={response.message}")

            if response.success:
                # Debug: เช็คค่าก่อนแปลง
                joint_angles = response.joint_angles
                self.ros_node.get_logger().info(f"Raw Joint Angles from IK: {joint_angles}")

                # ตรวจสอบว่า joint_angles มีค่าและจำนวนพอเพียง
                if not joint_angles or len(joint_angles) != len(self.joint_names):
                    self.ros_node.get_logger().error(f"Invalid joint angles received: {joint_angles}")
                    return  # ไม่ส่งคำสั่งหากค่าไม่ถูกต้อง

                # แปลงเป็น degrees หากอยู่ในช่วงของ radians
                if max(map(abs, joint_angles)) <= math.pi:  # ถ้าค่าสูงสุดไม่เกิน pi แสดงว่าเป็น radians
                    joint_angles_deg = [math.degrees(angle) for angle in joint_angles]
                    self.ros_node.get_logger().info("Detected radians, converting to degrees")
                else:
                    joint_angles_deg = joint_angles  # ใช้ค่าเดิมหากเป็น degrees อยู่แล้ว

                self.ros_node.get_logger().info(f"Converted Joint Angles (deg): {joint_angles_deg}")

                # ✅ อัปเดตค่าลง self.joint_values
                for i, joint in enumerate(self.joint_names):
                    self.joint_values[joint] = joint_angles_deg[i]

                # Debug: เช็คค่าก่อนส่งคำสั่ง
                self.ros_node.get_logger().info(f"Final Joint Values to Send: {self.joint_values}")

                # ✅ อัปเดต UI ให้แสดงค่าล่าสุด
                for i, joint in enumerate(self.joint_names):
                    self.joint_value_labels[joint].config(text=f"{joint_angles_deg[i]:.2f}°")

                # ✅ ส่งค่าไปยัง ROS /set_position
                self.send_joint_angles()

            else:
                self.ros_node.get_logger().error(f"IK solution failed: {response.message}")

        except Exception as e:
            self.ros_node.get_logger().error(f"Error handling SolveIK response: {str(e)}")
            import traceback
            self.ros_node.get_logger().error(traceback.format_exc())

        
    def navigate_joint_selection(self, direction):
        """Navigate joint selection with keyboard"""
        current_index = self.joint_names.index(self.joint_selector.get())
        new_index = (current_index + direction) % len(self.joint_names)
        self.joint_selector.set(self.joint_names[new_index])
    
    def update_velocity_from_slider(self, value):
        """Update velocity value from slider"""
        try:
            self.velocity = float(value)
            self.velocity_value_label.config(text=f"Current: {self.velocity:.2f}")
        except ValueError:
            pass
    
    def adjust_velocity(self, amount):
        """Adjust velocity by the specified amount"""
        new_velocity = max(0.0, min(2.0, self.velocity + amount))
        self.velocity = new_velocity
        self.speed_slider.set(new_velocity)
        self.velocity_value_label.config(text=f"Current: {self.velocity:.2f}")


def main():
    rclpy.init()
    node = Node("robot_control_ui")
    root = ttk.Window(themename="superhero")
    root.title("Robot Control UI")
    app = RobotControlUI(root, node)

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(100, ros_spin)
    
    root.after(100, ros_spin)
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()