import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
import threading
import tkinter as tk
from tkinter import font as tkfont

# Global state dictionary accessed by the GUI
gui_state = {
    'bot1': {'x': 0.0, 'y': 0.0, 'state': 'PAUSED'},
    'bot2': {'x': 0.0, 'y': 0.0, 'state': 'PAUSED'},
    'bot3': {'x': 0.0, 'y': 0.0, 'state': 'PAUSED'},
    'green_found': False,
    'green_x': 0.0,
    'green_y': 0.0,
}

class SwarmGuiNode(Node):
    def __init__(self):
        super().__init__('swarm_gui_server')
        
        # Publisher for global swarm control
        self.control_pub = self.create_publisher(String, '/swarm_control', 10)
        
        # Subscriptions for bot odometry to show coordinates in GUI
        self.create_subscription(Odometry, '/bot1/odom', lambda msg: self._odom_cb(msg, 'bot1'), 10)
        self.create_subscription(Odometry, '/bot2/odom', lambda msg: self._odom_cb(msg, 'bot2'), 10)
        self.create_subscription(Odometry, '/bot3/odom', lambda msg: self._odom_cb(msg, 'bot3'), 10)
        
        # Subscription for green cube location
        self.create_subscription(Point, '/green_cube_location', self._green_cb, 10)
        
        # Client for resetting Gazebo world
        self.reset_client = self.create_client(Empty, '/reset_world')
        
        # Subscription to monitor bot states
        self.create_subscription(String, '/bot1/state', lambda msg: self._state_cb(msg, 'bot1'), 10)
        self.create_subscription(String, '/bot2/state', lambda msg: self._state_cb(msg, 'bot2'), 10)
        self.create_subscription(String, '/bot3/state', lambda msg: self._state_cb(msg, 'bot3'), 10)
        
        self.get_logger().info('Swarm GUI Desktop Node initialized.')

    def _odom_cb(self, msg, bot_name):
        gui_state[bot_name]['x'] = round(msg.pose.pose.position.x, 2)
        gui_state[bot_name]['y'] = round(msg.pose.pose.position.y, 2)

    def _state_cb(self, msg, bot_name):
        gui_state[bot_name]['state'] = msg.data

    def _green_cb(self, msg):
        gui_state['green_found'] = True
        gui_state['green_x'] = round(msg.x, 2)
        gui_state['green_y'] = round(msg.y, 2)

    def publish_control(self, command):
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)
        if command == 'RESET':
            gui_state['green_found'] = False
            gui_state['green_x'] = 0.0
            gui_state['green_y'] = 0.0
            for bot in ['bot1', 'bot2', 'bot3']:
                gui_state[bot]['state'] = 'PAUSED'
            self.trigger_gazebo_reset()

    def trigger_gazebo_reset(self):
        if self.reset_client.service_is_ready():
            req = Empty.Request()
            self.reset_client.call_async(req)
            self.get_logger().info('Sent Gazebo reset_world request.')
        else:
            self.get_logger().warn('/reset_world service not ready/available.')


# ── Desktop GUI Implementation ──────────────────────────────────────────────
class SwarmApp(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.title("Antbot Swarm Control Hub")
        self.geometry("820x420")
        self.configure(bg="#111827") # Dark background (slate-900)
        
        # Create premium custom fonts
        self.title_font = tkfont.Font(family="Helvetica", size=20, weight="bold")
        self.subtitle_font = tkfont.Font(family="Helvetica", size=11, weight="normal")
        self.header_font = tkfont.Font(family="Helvetica", size=13, weight="bold")
        self.body_font = tkfont.Font(family="Helvetica", size=11, weight="normal")
        self.bold_body_font = tkfont.Font(family="Helvetica", size=11, weight="bold")
        
        self.setup_ui()
        self.update_loop()

    def setup_ui(self):
        # 1. Header Frame
        header_frame = tk.Frame(self, bg="#111827", pady=15)
        header_frame.pack(fill="x")
        
        title_lbl = tk.Label(
            header_frame, 
            text="ANTBOT SWARM CONTROL HUB", 
            fg="#818cf8", 
            bg="#111827", 
            font=self.title_font
        )
        title_lbl.pack()
        
        subtitle_lbl = tk.Label(
            header_frame, 
            text="Modify obstacles or goal in Gazebo, then use the desktop controls to operate the swarm.", 
            fg="#94a3b8", 
            bg="#111827", 
            font=self.subtitle_font
        )
        subtitle_lbl.pack(pady=4)

        # 2. Control Actions Frame
        ctrl_frame = tk.Frame(
            self, 
            bg="#1f2937", 
            highlightbackground="#374151", 
            highlightthickness=1, 
            padx=20, 
            pady=15
        )
        ctrl_frame.pack(fill="x", padx=20, pady=10)
        
        # Center the buttons
        ctrl_inner = tk.Frame(ctrl_frame, bg="#1f2937")
        ctrl_inner.pack(anchor="center")
        
        btn_start = tk.Button(
            ctrl_inner, 
            text="▶ START SWARM", 
            bg="#10b981", 
            fg="white", 
            font=self.bold_body_font,
            activebackground="#059669", 
            activeforeground="white",
            relief="flat", 
            padx=25, 
            pady=10,
            cursor="hand2",
            command=lambda: self.ros_node.publish_control("START")
        )
        btn_start.pack(side="left", padx=10)

        btn_stop = tk.Button(
            ctrl_inner, 
            text="⏸ STOP SWARM", 
            bg="#ef4444", 
            fg="white", 
            font=self.bold_body_font,
            activebackground="#dc2626", 
            activeforeground="white",
            relief="flat", 
            padx=25, 
            pady=10,
            cursor="hand2",
            command=lambda: self.ros_node.publish_control("STOP")
        )
        btn_stop.pack(side="left", padx=10)

        btn_reset = tk.Button(
            ctrl_inner, 
            text="🔄 RESET ALL", 
            bg="#f59e0b", 
            fg="white", 
            font=self.bold_body_font,
            activebackground="#d97706", 
            activeforeground="white",
            relief="flat", 
            padx=25, 
            pady=10,
            cursor="hand2",
            command=lambda: self.ros_node.publish_control("RESET")
        )
        btn_reset.pack(side="left", padx=10)

        # 3. Target Cube Status Panel
        self.target_frame = tk.Frame(
            self, 
            bg="#1e293b", 
            highlightbackground="#3b82f6", 
            highlightthickness=1, 
            padx=20, 
            pady=15
        )
        self.target_frame.pack(fill="x", padx=20, pady=10)
        
        self.target_title = tk.Label(
            self.target_frame, 
            text="Goal", 
            fg="#e2e8f0", 
            bg="#1e293b", 
            font=self.header_font
        )
        self.target_title.pack(side="left")
        
        self.target_badge = tk.Label(
            self.target_frame, 
            text="SEARCHING", 
            bg="#ef4444", 
            fg="white", 
            font=self.bold_body_font,
            padx=15, 
            pady=4
        )
        self.target_badge.pack(side="right")

        # 4. Robots Details Frame
        bots_container = tk.Frame(self, bg="#111827")
        bots_container.pack(fill="both", expand=True, padx=20, pady=10)
        
        self.bot_labels = {}
        
        for i, bot in enumerate(['bot1', 'bot2', 'bot3'], 1):
            card = tk.Frame(
                bots_container, 
                bg="#1f2937", 
                highlightbackground="#374151", 
                highlightthickness=1,
                padx=15,
                pady=20
            )
            # Grid layout to distribute columns evenly
            card.grid(row=0, column=i-1, sticky="nsew", padx=8, pady=5)
            bots_container.grid_columnconfigure(i-1, weight=1)
            
            # Bot header with status
            top_row = tk.Frame(card, bg="#1f2937")
            top_row.pack(fill="x", pady=5)
            
            bot_title = tk.Label(
                top_row, 
                text=f"Bot {i}", 
                fg="#c7d2fe", 
                bg="#1f2937", 
                font=self.header_font
            )
            bot_title.pack(side="left")
            
            status_badge = tk.Label(
                top_row, 
                text="PAUSED", 
                bg="#4b5563", 
                fg="#e5e7eb", 
                font=tkfont.Font(family="Helvetica", size=9, weight="bold"),
                padx=8, 
                pady=2
            )
            status_badge.pack(side="right")
            
            self.bot_labels[bot] = {
                'badge': status_badge
            }

    def get_badge_colors(self, state):
        state = state.upper()
        if state == 'PAUSED': return {"bg": "#4b5563", "fg": "#f3f4f6"}
        if state == 'SEARCH': return {"bg": "#d97706", "fg": "#fef08a"}
        if state == 'APPROACH': return {"bg": "#7c3aed", "fg": "#ddd6fe"}
        if state == 'STOPPED': return {"bg": "#059669", "fg": "#a7f3d0"}
        if state == 'GO_TO_TARGET': return {"bg": "#2563eb", "fg": "#bfdbfe"}
        return {"bg": "#4b5563", "fg": "#f3f4f6"}

    def update_loop(self):
        # Update each robot card UI elements
        for bot in ['bot1', 'bot2', 'bot3']:
            data = gui_state[bot]
            lbls = self.bot_labels[bot]
            
            badge_config = self.get_badge_colors(data['state'])
            lbls['badge'].configure(
                text=data['state'].upper(),
                bg=badge_config['bg'],
                fg=badge_config['fg']
            )

        # Update green target status UI elements
        if gui_state['green_found']:
            self.target_badge.configure(text="FOUND", bg="#10b981", fg="white")
            self.target_frame.configure(highlightbackground="#10b981")
        else:
            self.target_badge.configure(text="SEARCHING", bg="#ef4444", fg="white")
            self.target_frame.configure(highlightbackground="#3b82f6")

        # Schedule next update in 100ms
        self.after(100, self.update_loop)


def main(args=None):
    rclpy.init(args=args)
    ros_node = SwarmGuiNode()
    
    # Run the ROS 2 executor in a daemon thread so it does not block the Tkinter main loop
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()
    
    # Run the Tkinter desktop dashboard application on the main thread
    app = SwarmApp(ros_node)
    
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
