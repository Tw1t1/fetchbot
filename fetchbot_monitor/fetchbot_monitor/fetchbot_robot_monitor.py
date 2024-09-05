import rclpy
from rclpy.node import Node
from fetchbot_interfaces.msg import Heading, BallInfo, Collision, Force
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64
import tkinter as tk
from tkinter import ttk
from datetime import datetime
from PIL import Image as PILImage, ImageTk
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class FetchBotDisplay(Node):
    def __init__(self):
        super().__init__('fetchbot_monitor')
        self.topic_dict = {
            '/camera_sensor/image_raw': (Image, self.image_callback),
            '/image_out': (Image, self.image_callback),
            '/image_tuning': (Image, self.image_callback),
            '/detected_ball': (BallInfo, self.ball_info_callback),
            '/follow_ball': (Heading, self.heading_callback),
            '/follow_ball/status': (String, self.string_callback),
            'claw_cmd': (String, self.string_callback),
            'position': (Float64, self.float64_callback),
            'grab_ball/status': (String, self.string_callback),
            'avoid': (Heading, self.collision_callback),
            '/scan': (LaserScan, self.laser_scan_callback),
            'force': (Force, self.force_callback),
            'runaway': (Heading, self.heading_callback),
            '/diff_cont/cmd_vel_unstamped': (Twist, self.twist_callback),
            'wander': (Heading, self.heading_callback),
            'detect_ball_wander_inhibitor': (Heading, self.heading_callback),
            'follow_ball_wander_suppressor': (Heading, self.heading_callback),
            'orient_home_wander_suppressor': (Heading, self.heading_callback),
            'avoid_runaway_suppressor': (Heading, self.heading_callback),
            'orient_home_detect_ball_inhibitor': (BallInfo, self.ball_info_callback),
            'grab_ball_follow_ball_inhibitor': (Heading, self.heading_callback),
        }
        self.subscribers = {}
        self.setup_gui()
        self.setup_subscribers()

    def setup_gui(self):
        self.window = tk.Tk()
        self.window.title("FetchBot Monitor")
        self.window.geometry("1200x800")

        main_frame = ttk.Frame(self.window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.window.columnconfigure(0, weight=1)
        self.window.rowconfigure(0, weight=1)

        self.text_widgets = {}
        self.image_labels = {}
        self.plot_widgets = {}

        # Column for images
        image_frame = ttk.Frame(main_frame)
        image_frame.grid(row=0, column=0, sticky=(tk.N, tk.S), padx=5)
        for i, topic in enumerate(['/camera_sensor/image_raw', '/image_out', '/image_tuning']):
            frame = ttk.LabelFrame(image_frame, text=topic, padding="5")
            frame.grid(row=i, column=0, pady=5, sticky=(tk.W, tk.E))
            label = ttk.Label(frame)
            label.pack(expand=True, fill='both')
            self.image_labels[topic] = label

        # Column for scan and force
        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, sticky=(tk.N, tk.S), padx=5)
        for i, (topic, plot_type) in enumerate([('/scan', 'polar'), ('force', 'arrow')]):
            frame = ttk.LabelFrame(plot_frame, text=topic, padding="5")
            frame.grid(row=i, column=0, pady=5, sticky=(tk.W, tk.E))
            fig, ax = plt.subplots(figsize=(4, 3), subplot_kw=dict(projection='polar') if plot_type == 'polar' else {})
            canvas = FigureCanvasTkAgg(fig, master=frame)
            canvas_widget = canvas.get_tk_widget()
            canvas_widget.pack(expand=True, fill='both')
            self.plot_widgets[topic] = (fig, ax, canvas)

        # Columns for text widgets
        text_frame = ttk.Frame(main_frame)
        text_frame.grid(row=0, column=2, columnspan=2, sticky=(tk.N, tk.S, tk.E, tk.W))
        text_topics = [topic for topic in self.topic_dict if topic not in self.image_labels and topic not in self.plot_widgets]
        for i, topic in enumerate(text_topics):
            frame = ttk.LabelFrame(text_frame, text=topic, padding="5")
            frame.grid(row=i // 2, column=i % 2, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
            text_frame.columnconfigure(i % 2, weight=1)
            text_frame.rowconfigure(i // 2, weight=1)
            text_widget = tk.Text(frame, height=4, width=30)
            text_widget.pack(expand=True, fill='both')
            self.text_widgets[topic] = text_widget

        for i in range(4):
            main_frame.columnconfigure(i, weight=1)
        main_frame.rowconfigure(0, weight=1)

    def setup_subscribers(self):
        for topic, (msg_type, callback) in self.topic_dict.items():
            self.subscribers[topic] = self.create_subscription(
                msg_type,
                topic,
                lambda msg, cb=callback, t=topic: cb(t, msg),
                10)

    def image_callback(self, topic, msg):
        try:
            np_arr = np.array(msg.data, dtype=np.uint8)
            image_np = np_arr.reshape((msg.height, msg.width, -1))
            if image_np.shape[2] == 1:
                image_np = np.repeat(image_np, 3, axis=2)
            elif image_np.shape[2] == 4:
                image_np = image_np[:, :, :3]
            image = PILImage.fromarray(image_np)
            image = image.resize((200, 150), PILImage.LANCZOS)
            tk_image = ImageTk.PhotoImage(image)
            self.image_labels[topic].config(image=tk_image)
            self.image_labels[topic].image = tk_image
        except Exception as e:
            self.get_logger().error(f"Error processing image on topic {topic}: {str(e)}")

    def update_text_widget(self, topic, content):
        if topic in self.text_widgets:
            self.text_widgets[topic].delete('1.0', tk.END)
            self.text_widgets[topic].insert(tk.END, content)

    def heading_callback(self, topic, msg):
        content = f"distance: {msg.distance:.2f}\nangle: {msg.angle:.2f}"
        self.update_text_widget(topic, content)

    def ball_info_callback(self, topic, msg):
        content = f"x: {msg.pos_x:.2f}\ny: {msg.pos_y:.2f}\nsize: {msg.size:.2f}"
        self.update_text_widget(topic, content)

    def string_callback(self, topic, msg):
        self.update_text_widget(topic, msg.data)

    def float64_callback(self, topic, msg):
        self.update_text_widget(topic, f"Value: {msg.data:.2f}")

    def collision_callback(self, topic, msg):
        content = f"distance: {msg.distance:.2f}\nangle: {msg.angle:.2f}"
        self.update_text_widget(topic, content)
        # content = f"range: {msg.range:.2f}\nangle: {msg.angle:.2f}"
        # self.update_text_widget(topic, content)

    def laser_scan_callback(self, topic, msg):
        fig, ax, canvas = self.plot_widgets[topic]
        ax.clear()
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ax.plot(angles, msg.ranges)
        ax.set_theta_zero_location("N")
        ax.set_title("LaserScan")
        canvas.draw()

    def force_callback(self, topic, msg):
        fig, ax, canvas = self.plot_widgets[topic]
        ax.clear()
        ax.arrow(0, 0, msg.direction, msg.magnitude, head_width=0.1, head_length=0.1)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_title(f"Force: Mag={msg.magnitude:.2f}, Dir={msg.direction:.2f}")
        canvas.draw()

    def twist_callback(self, topic, msg):
        content = f"Linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}\n"
        content += f"Angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}"
        self.update_text_widget(topic, content)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.window.update()

def main(args=None):
    rclpy.init(args=args)
    node = FetchBotDisplay()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from fetchbot_interfaces.msg import Heading, BallInfo, Collision, Force
# from sensor_msgs.msg import Image, LaserScan
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String, Float64
# import tkinter as tk
# from tkinter import ttk
# from datetime import datetime
# from PIL import Image as PILImage, ImageTk
# import numpy as np
# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# class ResizableFrame(ttk.Frame):
#     def __init__(self, parent, title, *args, **kwargs):
#         super().__init__(parent, *args, **kwargs)
#         self.title = title
#         self.columnconfigure(0, weight=1)
#         self.rowconfigure(1, weight=1)

#         title_frame = ttk.Frame(self)
#         title_frame.grid(row=0, column=0, sticky="ew")
#         ttk.Label(title_frame, text=title).pack(side="left", padx=5, pady=2)
        
#         self.content_frame = ttk.Frame(self)
#         self.content_frame.grid(row=1, column=0, sticky="nsew")

#         self.resize_handle = ttk.Label(self, text="⋮")
#         self.resize_handle.grid(row=2, column=0, sticky="se")
#         self.resize_handle.bind("<ButtonPress-1>", self.start_resize)
#         self.resize_handle.bind("<B1-Motion>", self.do_resize)

#     def start_resize(self, event):
#         self.start_x = event.x
#         self.start_y = event.y

#     def do_resize(self, event):
#         width = self.winfo_width() + event.x - self.start_x
#         height = self.winfo_height() + event.y - self.start_y
#         self.config(width=max(100, width), height=max(50, height))

# class FetchBotDisplay(Node):
#     def __init__(self):
#         super().__init__('fetchbot_monitor')
#         self.topic_dict = {
#             '/camera_sensor/image_raw': (Image, self.image_callback),
#             '/image_in': (Image, self.image_callback),
#             '/image_tuning': (Image, self.image_callback),
#             '/detected_ball': (BallInfo, self.ball_info_callback),
#             '/follow_ball': (Heading, self.heading_callback),
#             '/follow_ball/status': (String, self.string_callback),
#             'claw_cmd': (String, self.string_callback),
#             'position': (Float64, self.float64_callback),
#             'grab_ball/status': (String, self.string_callback),
#             'collision': (Collision, self.collision_callback),
#             '/scan': (LaserScan, self.laser_scan_callback),
#             'force': (Force, self.force_callback),
#             'runaway': (Heading, self.heading_callback),
#             '/diff_cont/cmd_vel_unstamped': (Twist, self.twist_callback),
#             'wander': (Heading, self.heading_callback),
#             'detect_ball_wander_inhibitor': (Heading, self.heading_callback),
#             'follow_ball_wander_suppressor': (Heading, self.heading_callback),
#             'orient_home_wander_suppressor': (Heading, self.heading_callback),
#             'avoid_runaway_suppressor': (Heading, self.heading_callback),
#             'orient_home_detect_ball_inhibitor': (BallInfo, self.ball_info_callback),
#             'grab_ball_follow_ball_inhibitor': (Heading, self.heading_callback),
#         }
#         self.subscribers = {}
#         self.frames = {}
#         self.setup_gui()
#         self.setup_subscribers()

#     def setup_gui(self):
#         self.window = tk.Tk()
#         self.window.title("FetchBot Monitor")
#         self.window.geometry("1200x800")

#         control_frame = ttk.Frame(self.window, padding="10")
#         control_frame.pack(fill='x')
#         ttk.Button(control_frame, text="Reset Layout", command=self.reset_layout).pack(side='left', padx=5)

#         self.main_frame = ttk.Frame(self.window)
#         self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

#         self.create_topic_frames()

#     def create_topic_frames(self):
#         for i, topic in enumerate(self.topic_dict):
#             frame = ResizableFrame(self.main_frame, topic)
#             frame.grid(row=i // 4, column=i % 4, padx=5, pady=5, sticky="nsew")
#             self.frames[topic] = self.setup_topic_frame(frame, topic)

#         for i in range(4):
#             self.main_frame.columnconfigure(i, weight=1)
#         for i in range((len(self.topic_dict) + 3) // 4):
#             self.main_frame.rowconfigure(i, weight=1)

#     def setup_topic_frame(self, frame, topic):
#         content_frame = frame.content_frame
#         if self.topic_dict[topic][0] == Image:
#             widget = ttk.Label(content_frame)
#             widget.pack(expand=True, fill='both')
#         elif topic == '/scan':
#             fig, ax = plt.subplots(figsize=(3, 2), subplot_kw=dict(projection='polar'))
#             canvas = FigureCanvasTkAgg(fig, master=content_frame)
#             canvas_widget = canvas.get_tk_widget()
#             canvas_widget.pack(expand=True, fill='both')
#             widget = (fig, ax, canvas)
#         elif topic == 'force':
#             fig, ax = plt.subplots(figsize=(3, 2))
#             canvas = FigureCanvasTkAgg(fig, master=content_frame)
#             canvas_widget = canvas.get_tk_widget()
#             canvas_widget.pack(expand=True, fill='both')
#             widget = (fig, ax, canvas)
#         else:
#             widget = tk.Text(content_frame, height=3, width=30)
#             widget.pack(expand=True, fill='both')

#         time_label = ttk.Label(content_frame, text="Last update: Never", font=('Helvetica', 8))
#         time_label.pack(anchor='w')

#         return {'widget': widget, 'time_label': time_label}

#     def reset_layout(self):
#         for widget in self.main_frame.winfo_children():
#             widget.destroy()
#         self.frames.clear()
#         self.create_topic_frames()

#     def setup_subscribers(self):
#         for topic, (msg_type, callback) in self.topic_dict.items():
#             self.subscribers[topic] = TopicSubscriber(self, topic, msg_type, callback)

#     def update_time_label(self, topic):
#         current_time = datetime.now().strftime("%H:%M:%S")
#         self.frames[topic]['time_label'].config(text=f"Last: {current_time}")

#     def update_text_widget(self, topic, content):
#         if topic in self.frames:
#             widget = self.frames[topic]['widget']
#             if isinstance(widget, tk.Text):
#                 widget.delete('1.0', tk.END)
#                 widget.insert(tk.END, content)
#         self.update_time_label(topic)

#     def image_callback(self, topic, msg):
#         try:
#             # Convert the image data to a numpy array
#             np_arr = np.array(msg.data, dtype=np.uint8)

#             # Reshape the array to the correct dimensions
#             image_np = np_arr.reshape((msg.height, msg.width, -1))

#             # Convert to RGB if necessary
#             if image_np.shape[2] == 1:  # Grayscale
#                 image_np = np.repeat(image_np, 3, axis=2)
#             elif image_np.shape[2] == 4:  # RGBA
#                 image_np = image_np[:, :, :3]

#             # Convert numpy array to PIL Image
#             image = PILImage.fromarray(image_np)
#             image = image.resize((150, 100), PILImage.LANCZOS)
#             tk_image = ImageTk.PhotoImage(image)
#             self.frames[topic]['widget'].config(image=tk_image)
#             self.frames[topic]['widget'].image = tk_image
#             self.update_time_label(topic)
#         except Exception as e:
#             self.get_logger().error(f"Error processing image on topic {topic}: {str(e)}")

#     def heading_callback(self, topic, msg):
#         content = f"Heading:\ndistance: {msg.distance:.2f}\nangle: {msg.angle:.2f}"
#         self.update_text_widget(topic, content)

#     def ball_info_callback(self, topic, msg):
#         content = f"BallInfo:\nx: {msg.pos_x:.2f}\ny: {msg.pos_y:.2f}\nsize: {msg.size:.2f}"
#         self.update_text_widget(topic, content)

#     def string_callback(self, topic, msg):
#         self.update_text_widget(topic, msg.data)

#     def float64_callback(self, topic, msg):
#         self.update_text_widget(topic, f"Value: {msg.data:.2f}")

#     def collision_callback(self, topic, msg):
#         content = f"Collision:\nrange: {msg.range:.2f}\nangle: {msg.angle:.2f}"
#         self.update_text_widget(topic, content)

#     def laser_scan_callback(self, topic, msg):
#         if topic in self.frames:
#             widget = self.frames[topic]['widget']
#             if isinstance(widget, tuple) and len(widget) == 3:
#                 fig, ax, canvas = widget
#                 ax.clear()
#                 angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
#                 ax.plot(angles, msg.ranges)
#                 ax.set_theta_zero_location("N")
#                 ax.set_title("LaserScan")
#                 canvas.draw()
#         self.update_time_label(topic)

#     def force_callback(self, topic, msg):
#         if topic in self.frames:
#             widget = self.frames[topic]['widget']
#             if isinstance(widget, tuple) and len(widget) == 3:
#                 fig, ax, canvas = widget
#                 ax.clear()
#                 ax.arrow(0, 0, msg.direction, msg.magnitude, head_width=0.1, head_length=0.1)
#                 ax.set_xlim(-1, 1)
#                 ax.set_ylim(-1, 1)
#                 ax.set_title(f"Force: Mag={msg.magnitude:.2f}, Dir={msg.direction:.2f}")
#                 canvas.draw()
#         self.update_time_label(topic)

#     def twist_callback(self, topic, msg):
#         content = f"Twist:\nLinear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}\n"
#         content += f"Angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}"
#         self.update_text_widget(topic, content)

#     def run(self):
#         while rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.1)
#             self.window.update()

# class TopicSubscriber:
#     def __init__(self, node, topic_name, msg_type, callback):
#         self.topic_name = topic_name
#         self.subscription = node.create_subscription(
#             msg_type,
#             topic_name,
#             lambda msg: callback(topic_name, msg),
#             10)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FetchBotDisplay()
#     try:
#         node.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()