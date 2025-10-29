#!/usr/bin/env python3
"""
LDLidar Statistics Logger Node with Tkinter GUI

Subscribes to /ldlidar_stats topic and displays statistics in a GUI window.
Also logs to a timestamped text file for persistence.

This is a lightweight GUI implementation optimized for Raspberry Pi with limited resources.
Uses built-in Tkinter (no additional packages required).

Features:
- Real-time statistics display in scrollable window
- Pause/Resume functionality to reduce CPU load
- Clear display button
- Automatic memory management (limits display lines)
- Persistent file logging with timestamps
- Thread-safe GUI updates

Author: Auto-generated for Raspberry Pi LDLidar ROS2 project
License: Apache License 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
from datetime import datetime
import threading

# Import tkinter with error handling for display issues
try:
    import tkinter as tk
    from tkinter import scrolledtext
except ImportError as e:
    print(f"Error importing tkinter: {e}")
    print("Please install python3-tk package")
    sys.exit(1)
except Exception as e:
    print(f"Error initializing tkinter: {e}")
    print("Check if DISPLAY is set correctly and X server is running")
    sys.exit(1)


class LidarStatsLoggerGUI(Node):
    def __init__(self):
        super().__init__('ldlidar_stats_logger_gui')
        
        # Log display information
        display = os.environ.get('DISPLAY', 'not set')
        self.get_logger().info(f'DISPLAY environment: {display}')
        
        # Declare parameters
        self.declare_parameter('log_file_path', '/workspace/logs')
        self.declare_parameter('max_display_lines', 500)
        
        # Get parameters
        log_file_path_param = self.get_parameter('log_file_path').value
        self.log_file_path = os.path.expanduser(log_file_path_param)
        self.max_lines = self.get_parameter('max_display_lines').value
        
        # Create log directory
        try:
            os.makedirs(self.log_file_path, exist_ok=True)
            self.get_logger().info(f'Log directory: {self.log_file_path}')
        except Exception as e:
            self.get_logger().error(f'Could not create directory "{self.log_file_path}": {e}. Falling back to /tmp')
            self.log_file_path = '/tmp'
            os.makedirs(self.log_file_path, exist_ok=True)
        
        # Create timestamped log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_file_path, f'lidar_stats_{timestamp}.txt')
        
        try:
            with open(self.log_file, 'w') as f:
                f.write('LDLidar Statistics Log\n')
                f.write(f'Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('Format: Row - min: (dist, angle) - max: (dist, angle) - avg: dist\n')
                f.write('='*80 + '\n')
            self.get_logger().info(f'Log file: {self.log_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {e}')
            self.log_file = None
        
        # Subscribe to stats topic
        self.subscription = self.create_subscription(
            String,
            '/ldlidar_stats',
            self.stats_callback,
            10
        )
        
        self.get_logger().info('Stats logger GUI initialized')
        
        # Line counter for memory management
        self.line_count = 0
        self.paused = False
        
        # Create GUI
        self.create_gui()
        
    def create_gui(self):
        """Create lightweight Tkinter GUI window"""
        try:
            self.root = tk.Tk()
        except tk.TclError as e:
            self.get_logger().error(f"Failed to create Tk window: {e}")
            self.get_logger().error("Check if DISPLAY is set and X server is running")
            self.get_logger().error(f"Current DISPLAY: {os.environ.get('DISPLAY', 'not set')}")
            raise
        
        self.root.title('LDLidar Statistics Logger')
        self.root.geometry('750x550')
        
        # Force window to appear on top initially
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)
        
        # Configure window to handle closing properly
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Info label
        info_text = f"Log file: {os.path.basename(self.log_file) if self.log_file else 'None'}"
        info_label = tk.Label(main_frame, text=info_text, font=('Arial', 9), anchor='w')
        info_label.pack(fill=tk.X, pady=(0, 5))
        
        # Create scrolled text widget
        self.text_area = scrolledtext.ScrolledText(
            main_frame,
            wrap=tk.WORD,
            width=90,
            height=30,
            font=('Courier', 9)
        )
        self.text_area.pack(fill=tk.BOTH, expand=True)
        
        # Button frame
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=5)
        
        # Clear button
        clear_btn = tk.Button(
            btn_frame,
            text="Clear Display",
            command=self.clear_display,
            width=12
        )
        clear_btn.pack(side=tk.LEFT, padx=5)
        
        # Pause/Resume button
        self.pause_btn = tk.Button(
            btn_frame,
            text="Pause",
            command=self.toggle_pause,
            width=12
        )
        self.pause_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = tk.Label(
            btn_frame,
            text="Status: Running",
            font=('Arial', 9),
            fg='green'
        )
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # Add header to text area
        header = "LDLidar Statistics Logger\n"
        header += f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        header += f"Log directory: {self.log_file_path}\n"
        header += "="*80 + "\n\n"
        self.text_area.insert(tk.END, header)
        self.line_count = header.count('\n')
        
    def stats_callback(self, msg):
        """Callback for stats messages"""
        if self.paused:
            # Still log to file even when paused
            self._log_to_file(msg.data)
            return
        
        timestamp = datetime.now().strftime('%H:%M:%S')
        stats_text = f"[{timestamp}]\n{msg.data}\n" + "-"*80 + "\n"
        
        # Update GUI (thread-safe)
        try:
            self.root.after(0, self.update_display, stats_text)
        except tk.TclError:
            # Window might be closing
            pass
        
        # Log to file
        self._log_to_file(msg.data)
    
    def _log_to_file(self, data):
        """Write data to log file"""
        if self.log_file is not None:
            try:
                with open(self.log_file, 'a') as f:
                    f.write(f"\n{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(data + '\n')
                    f.write('-'*80 + '\n')
                    f.flush()
                    os.fsync(f.fileno())
            except Exception as e:
                self.get_logger().error(f'Error writing to log file: {e}')
    
    def update_display(self, text):
        """Update display with memory management"""
        # Count new lines
        new_lines = text.count('\n')
        
        # If exceeding max lines, remove old ones
        if self.line_count + new_lines > self.max_lines:
            lines_to_remove = (self.line_count + new_lines) - self.max_lines
            self.text_area.delete(1.0, f"{lines_to_remove}.0")
            self.line_count -= lines_to_remove
        
        # Add new text
        self.text_area.insert(tk.END, text)
        self.text_area.see(tk.END)  # Auto-scroll to bottom
        self.line_count += new_lines
    
    def clear_display(self):
        """Clear the text display"""
        self.text_area.delete(1.0, tk.END)
        header = "LDLidar Statistics Logger\n"
        header += f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        header += f"Log directory: {self.log_file_path}\n"
        header += "="*80 + "\n\n"
        self.text_area.insert(tk.END, header)
        self.line_count = header.count('\n')
        self.get_logger().info('Display cleared')
    
    def toggle_pause(self):
        """Toggle pause state"""
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.config(text="Resume")
            self.status_label.config(text="Status: Paused", fg='orange')
            self.get_logger().info('Display paused (still logging to file)')
        else:
            self.pause_btn.config(text="Pause")
            self.status_label.config(text="Status: Running", fg='green')
            self.get_logger().info('Display resumed')
    
    def on_closing(self):
        """Handle window close event"""
        self.get_logger().info('Closing GUI window...')
        
        # Write closing message to log file
        if self.log_file is not None:
            try:
                with open(self.log_file, 'a') as f:
                    f.write('='*80 + '\n')
                    f.write(f'Ended: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                self.get_logger().info(f'Log file closed: {self.log_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to close log file: {e}')
        
        # Destroy the window
        self.root.quit()
        self.root.destroy()


def main(args=None):
    """Main function"""
    # Check DISPLAY variable early
    display = os.environ.get('DISPLAY', None)
    if not display:
        print("ERROR: DISPLAY environment variable is not set!")
        print("For WSL2 with VcXsrv, set DISPLAY to your Windows IP:0.0")
        print("Example: export DISPLAY=172.x.x.x:0.0")
        sys.exit(1)
    
    print(f"DISPLAY is set to: {display}")
    
    # Test if we can create a Tk window
    try:
        test_root = tk.Tk()
        test_root.withdraw()  # Hide the test window
        test_root.destroy()
        print("Tkinter display test: OK")
    except Exception as e:
        print(f"ERROR: Cannot connect to X server: {e}")
        print("Make sure VcXsrv/X server is running on Windows")
        print("Check firewall settings allow WSL2 connections")
        sys.exit(1)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        print("Creating GUI node...")
        node = LidarStatsLoggerGUI()
        
        print("Starting ROS2 spin thread...")
        # Run ROS2 spin in separate thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        
        print("Starting Tkinter main loop...")
        print("GUI window should appear now...")
        # Run Tkinter main loop in main thread
        node.root.mainloop()
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f'Error in main: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("Shutdown complete")


if __name__ == '__main__':
    main()
