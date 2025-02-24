import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
from serial.tools import list_ports


class SerialController:
    def __init__(self, app):
        self.app = app  # 主程序引用
        self.ser = None  # 串口对象
        self.port_list = []  # 可用端口列表

        # 串口参数默认值
        self.port_var = tk.StringVar(value='COM1')
        self.baud_var = tk.StringVar(value='115200')
        self.data_bits_var = tk.StringVar(value='8')
        self.stop_bits_var = tk.StringVar(value='1')
        self.parity_var = tk.StringVar(value='None')

        # 校验位映射表
        self.parity_mapping = {
            'None': serial.PARITY_NONE,
            'Even': serial.PARITY_EVEN,
            'Odd': serial.PARITY_ODD
        }

    def scan_ports(self):
        """扫描可用串口"""
        self.port_list = [port.device for port in list_ports.comports()]
        self.app.port_combobox['values'] = self.port_list
        if self.port_list:
            self.port_var.set(self.port_list[0])

    def connect(self):
        """建立串口连接"""
        if self.ser and self.ser.is_open:
            self.disconnect()
            return

        try:
            self.ser = serial.Serial(
                port=self.port_var.get(),
                baudrate=int(self.baud_var.get()),
                bytesize=int(self.data_bits_var.get()),
                stopbits=float(self.stop_bits_var.get()),
                parity=self.parity_mapping[self.parity_var.get()],
                timeout=1
            )
            self.app.update_status(" 已连接", 'green')
            self.app.log_text.insert(tk.END, f"成功连接到 {self.port_var.get()}\n")
        except Exception as e:
            messagebox.showerror(" 连接错误", f"连接失败: {str(e)}")
            self.app.update_status(" 未连接", 'red')

    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.app.update_status(" 未连接", 'red')
        self.app.log_text.insert(tk.END, "已断开连接\n")

    def send_cmd(self, value):
        """发送指令到串口"""
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning(" 警告", "请先连接串口")
            return

        try:
            self.ser.write(bytes([value]))
            self.app.log_text.insert(tk.END, f"发送指令: 0x{value:02X}\n")
            self.app.log_text.see(tk.END)
        except serial.SerialException as e:
            self.app.log_text.insert(tk.END, f"发送失败: {str(e)}\n")
            self.disconnect()


class AppGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("复成医疗-推杆测试上位机")
        self.root.geometry("750x500")

        self.serial_ctrl = SerialController(self)
        self.create_widgets()

    def create_widgets(self):
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 左侧控制面板
        control_frame = ttk.LabelFrame(main_frame, text="控制按钮")
        control_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.NSEW)

        # 创建6个控制按钮（2行3列）
        for i in range(1, 7):
            btn = ttk.Button(control_frame, text=f"控制发送 {i}")
            btn.grid(row=(i - 1) // 3, column=(i - 1) % 3, padx=5, pady=5, sticky=tk.NSEW)
            btn.bind('<ButtonPress>', lambda e, x=i: self.button_press(x))
            btn.bind('<ButtonRelease>', lambda e: self.button_release())
            btn.config(width=10)

            # 右侧串口设置面板
        serial_frame = ttk.LabelFrame(main_frame, text="串口配置")
        serial_frame.grid(row=0, column=1, padx=5, pady=5, sticky=tk.NSEW)

        # 端口选择
        ttk.Label(serial_frame, text="COM端口:").grid(row=0, column=0, sticky=tk.W)
        self.port_combobox = ttk.Combobox(serial_frame, textvariable=self.serial_ctrl.port_var)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=2)

        # 波特率选择
        ttk.Label(serial_frame, text="波特率:").grid(row=1, column=0, sticky=tk.W)
        baud_combobox = ttk.Combobox(serial_frame, values=['9600', '19200', '38400', '57600', '115200'],
                                     textvariable=self.serial_ctrl.baud_var)
        baud_combobox.grid(row=1, column=1, padx=5, pady=2)

        # 数据位选择
        ttk.Label(serial_frame, text="数据位:").grid(row=2, column=0, sticky=tk.W)
        data_combobox = ttk.Combobox(serial_frame, values=['5', '6', '7', '8'],
                                     textvariable=self.serial_ctrl.data_bits_var)
        data_combobox.grid(row=2, column=1, padx=5, pady=2)

        # 停止位选择
        ttk.Label(serial_frame, text="停止位:").grid(row=3, column=0, sticky=tk.W)
        stop_combobox = ttk.Combobox(serial_frame, values=['1', '1.5', '2'],
                                     textvariable=self.serial_ctrl.stop_bits_var)
        stop_combobox.grid(row=3, column=1, padx=5, pady=2)

        # 校验位选择
        ttk.Label(serial_frame, text="校验位:").grid(row=4, column=0, sticky=tk.W)
        parity_combobox = ttk.Combobox(serial_frame, values=['None', 'Even', 'Odd'],
                                       textvariable=self.serial_ctrl.parity_var)
        parity_combobox.grid(row=4, column=1, padx=5, pady=2)

        # 连接按钮
        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=5, column=0, columnspan=2, pady=5)

        # 日志显示区域
        log_frame = ttk.LabelFrame(main_frame, text="通信日志")
        log_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=tk.NSEW)

        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # 状态栏
        self.status_bar = ttk.Frame(self.root)
        self.status_bar.pack(fill=tk.X, padx=10, pady=2)
        self.status_label = ttk.Label(self.status_bar, text="状态: ")
        self.status_label.pack(side=tk.LEFT)
        self.status_led = ttk.Label(self.status_bar, background='red', width=2)
        self.status_led.pack(side=tk.LEFT, padx=5)

        # 配置网格布局权重
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=0)

        # 初始化扫描端口
        self.serial_ctrl.scan_ports()

    def toggle_connection(self):
        """切换连接状态"""
        if self.serial_ctrl.ser and self.serial_ctrl.ser.is_open:
            self.serial_ctrl.disconnect()
            self.connect_btn.config(text=" 连接")
        else:
            self.serial_ctrl.connect()
            self.connect_btn.config(text=" 断开")

    def button_press(self, btn_num):
        """按钮按下事件处理"""
        self.serial_ctrl.send_cmd(btn_num)

    def button_release(self):
        """按钮释放事件处理"""
        self.serial_ctrl.send_cmd(0x00)

    def update_status(self, text, color):
        """更新状态栏"""
        self.status_label.config(text=f" 状态: {text}")
        self.status_led.config(background=color)


if __name__ == "__main__":
    root = tk.Tk()
    app = AppGUI(root)
    root.mainloop()