import socket
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, simpledialog

ESPA_IP = "192.168.4.1"  # Default AP IP for ESP32
ESPA_PORT = 8888
sock = None
receive_thread_running = True
message_queue = queue.Queue()

# GUI globals
app = None
output_text = None
status_label = None
ip_var = None
port_var = None
receive_thread = None

def gui_print(message):
    """Print message to GUI text area"""
    if output_text:
        output_text.insert(tk.END, message + "\n")
        output_text.see(tk.END)

def update_status(message):
    """Update status label"""
    if status_label:
        status_label.config(text=f"Status: {message}")

def receive_data_thread_gui(s):
    global receive_thread_running
    buffer = ""
    try:
        while receive_thread_running:
            try:
                data = s.recv(1024)
                if not data:
                    gui_print("[INFO] Connection closed by server.")
                    message_queue.put(None)
                    receive_thread_running = False
                    break
                
                buffer += data.decode('utf-8', errors='ignore')
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        message_queue.put(line)

            except socket.timeout:
                continue
            except ConnectionResetError:
                gui_print("[ERROR] Connection reset by server.")
                message_queue.put(None)
                receive_thread_running = False
                break
            except Exception as e:
                if receive_thread_running:
                    gui_print(f"[ERROR] Error receiving data: {e}")
                message_queue.put(None)
                receive_thread_running = False
                break
            time.sleep(0.01)
    finally:
        receive_thread_running = False
        gui_print("[INFO] Receive thread stopped.")

def connect_to_espa_gui():
    global sock, receive_thread_running, receive_thread
    
    ip = ip_var.get() or ESPA_IP
    port = int(port_var.get() or ESPA_PORT)
    
    if sock:
        disconnect_from_espa_gui()
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        gui_print(f"[INFO] Attempting to connect to ESPA at {ip}:{port}...")
        update_status("Connecting...")
        sock.connect((ip, port))
        sock.settimeout(1.0)
        gui_print("[INFO] Connected to ESPA.")
        update_status("Connected")
        
        receive_thread_running = True
        receive_thread = threading.Thread(target=receive_data_thread_gui, args=(sock,))
        receive_thread.daemon = True
        receive_thread.start()
        return True
    except socket.timeout:
        gui_print("[ERROR] Connection attempt timed out.")
        update_status("Disconnected")
        sock = None
        return False
    except Exception as e:
        gui_print(f"[ERROR] Failed to connect: {e}")
        update_status("Disconnected")
        sock = None
        return False

def disconnect_from_espa_gui():
    global sock, receive_thread_running, receive_thread
    
    receive_thread_running = False
    if receive_thread and receive_thread.is_alive():
        receive_thread.join(timeout=1)
    if sock:
        sock.close()
        sock = None
    update_status("Disconnected")
    gui_print("[INFO] Disconnected from ESPA.")

def send_command_to_espa(command):
    if sock and receive_thread_running:
        try:
            sock.sendall((command + "\n").encode('utf-8'))
            gui_print(f"Sent: {command}")
        except Exception as e:
            gui_print(f"[ERROR] Error sending command: {e}")
            disconnect_from_espa_gui()
    else:
        gui_print("[INFO] Not connected. Please connect first.")

def process_message_queue():
    """Process messages from the queue and update GUI"""
    while not message_queue.empty():
        msg = message_queue.get()
        if msg is None:  # Connection closed
            disconnect_from_espa_gui()
            return
        gui_print(f"ESPA: {msg}")
    
    # Schedule next check
    if app:
        app.after(100, process_message_queue)

def send_params_command():
    """Get PID parameters from user and send PARAMS command"""
    try:
        kp = simpledialog.askfloat("PID Parameters", "Enter Kp:", initialvalue=50.0)
        if kp is None:
            return
        ki = simpledialog.askfloat("PID Parameters", "Enter Ki:", initialvalue=2.0)
        if ki is None:
            return
        kd = simpledialog.askfloat("PID Parameters", "Enter Kd:", initialvalue=40.0)
        if kd is None:
            return
        
        command = f"PARAMS {kp} {ki} {kd}"
        send_command_to_espa(command)
    except Exception as e:
        messagebox.showerror("Error", f"Invalid input: {e}")

def send_test_freq_command():
    """Get frequency from user and send TEST_FREQ command"""
    try:
        freq = simpledialog.askfloat("Test Frequency", "Enter frequency:", initialvalue=600)
        if freq is not None:
            send_command_to_espa(f"TEST_FREQ {freq}")
    except Exception as e:
        messagebox.showerror("Error", f"Invalid input: {e}")

def send_test_steps_command():
    """Get steps from user and send TEST_STEPS command"""
    try:
        steps = simpledialog.askinteger("Test Steps", "Enter steps:", initialvalue=100)
        if steps is not None:
            send_command_to_espa(f"TEST_STEPS {steps}")
    except Exception as e:
        messagebox.showerror("Error", f"Invalid input: {e}")

def create_gui(root):
    global output_text, status_label, ip_var, port_var
    
    root.title("ESPA Float Controller")
    root.geometry("800x600")
    
    # Main frame
    main_frame = ttk.Frame(root, padding="10")
    main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
    
    # Connection frame
    conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="5")
    conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
    
    ttk.Label(conn_frame, text="IP:").grid(row=0, column=0, sticky=tk.W)
    ip_var = tk.StringVar(value=ESPA_IP)
    ttk.Entry(conn_frame, textvariable=ip_var, width=15).grid(row=0, column=1, padx=(5, 10))
    
    ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky=tk.W)
    port_var = tk.StringVar(value=str(ESPA_PORT))
    ttk.Entry(conn_frame, textvariable=port_var, width=8).grid(row=0, column=3, padx=(5, 10))
    
    ttk.Button(conn_frame, text="Connect", command=connect_to_espa_gui).grid(row=0, column=4, padx=5)
    ttk.Button(conn_frame, text="Disconnect", command=disconnect_from_espa_gui).grid(row=0, column=5, padx=5)
    
    # Status
    status_label = ttk.Label(main_frame, text="Status: Disconnected")
    status_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
    
    # Commands frame
    cmd_frame = ttk.LabelFrame(main_frame, text="Commands", padding="5")
    cmd_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
    
    # Basic commands
    commands = [
        ("GO", "GO"),
        ("Send Data", "SEND_DATA"),
        ("Balance", "BALANCE"),
        ("Clear EEPROM", "CLEAR_EEPROM"),
        ("Switch Auto Mode", "SWITCH_AUTO_MODE"),
        ("Send Package", "SEND_PACKAGE"),
        ("OTA Update", "OTA_UPDATE"),
        ("Debug Mode", "DEBUG_MODE"),
        ("Status Request", "STATUS_REQ"),
        ("Home Motor", "HOME")
    ]
    
    for i, (label, command) in enumerate(commands):
        ttk.Button(cmd_frame, text=label, 
                  command=lambda cmd=command: send_command_to_espa(cmd)).grid(
                  row=i//2, column=i%2, sticky=(tk.W, tk.E), padx=2, pady=2)
    
    # Special commands with parameters
    ttk.Button(cmd_frame, text="Set PID Params", 
              command=send_params_command).grid(row=6, column=0, sticky=(tk.W, tk.E), padx=2, pady=2)
    ttk.Button(cmd_frame, text="Test Frequency", 
              command=send_test_freq_command).grid(row=6, column=1, sticky=(tk.W, tk.E), padx=2, pady=2)
    ttk.Button(cmd_frame, text="Test Steps", 
              command=send_test_steps_command).grid(row=7, column=0, sticky=(tk.W, tk.E), padx=2, pady=2)
    
    # Configure column weights for commands frame
    cmd_frame.columnconfigure(0, weight=1)
    cmd_frame.columnconfigure(1, weight=1)
    
    # Output frame
    output_frame = ttk.LabelFrame(main_frame, text="Output", padding="5")
    output_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
    
    # Text area with scrollbar
    output_text = scrolledtext.ScrolledText(output_frame, width=50, height=20, wrap=tk.WORD)
    output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
    
    # Clear button
    ttk.Button(output_frame, text="Clear Output", 
              command=lambda: output_text.delete(1.0, tk.END)).grid(row=1, column=0, pady=(5, 0))
    
    # Configure weights
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    main_frame.columnconfigure(1, weight=1)
    main_frame.rowconfigure(2, weight=1)
    output_frame.columnconfigure(0, weight=1)
    output_frame.rowconfigure(0, weight=1)
    
    # Initial message
    gui_print("--- ESPA Float Controller GUI ---")
    gui_print("Use the buttons to send commands to ESPA.")
    gui_print("Connect to ESPA first using the connection controls.")
    gui_print("Available commands:")
    gui_print("  GO, SEND_DATA, BALANCE, CLEAR_EEPROM, SWITCH_AUTO_MODE")
    gui_print("  SEND_PACKAGE, OTA_UPDATE, DEBUG_MODE, STATUS_REQ, HOME")
    gui_print("  PARAMS <Kp> <Ki> <Kd>, TEST_FREQ <freq>, TEST_STEPS <steps>")

def main():
    global app
    
    # Create GUI
    app = tk.Tk()
    create_gui(app)
    
    # Start message queue processing
    process_message_queue()
    
    # Handle window close
    def on_closing():
        global receive_thread_running
        receive_thread_running = False
        if sock:
            sock.close()
        app.destroy()
    
    app.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start GUI
    app.mainloop()

if __name__ == "__main__":
    main()
