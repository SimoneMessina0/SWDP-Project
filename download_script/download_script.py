import os
import sys
import subprocess

def ensure_venv():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    venv_python = os.path.join(script_dir, 'venv', 'Scripts', 'python.exe')
    
    if os.path.abspath(sys.executable) != os.path.abspath(venv_python) and os.path.exists(venv_python):
        print("🔄 Relaunching using local venv...")
        sys.exit(subprocess.call([venv_python] + sys.argv))

ensure_venv()

import numpy as np
import matplotlib.pyplot as plt
import serial
import pandas as pd
import struct
from datetime import datetime
from tkinter import Tk, filedialog, ttk, messagebox, StringVar, Label, Button
from serial.tools import list_ports

def convert_16bit_signed(lo, hi):
    combined = (hi.astype(np.uint16) << 8) | lo.astype(np.uint16)
    return combined.view(np.int16)

def conv_imu(arr):
    x = convert_16bit_signed(arr[:,0], arr[:,1]) / (2**15) * 2
    y = convert_16bit_signed(arr[:,2], arr[:,3]) / (2**15) * 2
    z = convert_16bit_signed(arr[:,4], arr[:,5]) / (2**15) * 2
    return x, y, z

def conv_gyro(arr):
    x = convert_16bit_signed(arr[:,0], arr[:,1]) / (2**15) * 250
    y = convert_16bit_signed(arr[:,2], arr[:,3]) / (2**15) * 250
    z = convert_16bit_signed(arr[:,4], arr[:,5]) / (2**15) * 250
    return x, y, z

def receive_and_save_data(ser, bin_filename, packet_size=4096, max_packets=2048*64):
    """Riceve pacchetti via seriale e li salva in binario."""
    with open(bin_filename, 'wb') as f:
        for packet_count in range(max_packets):
            data = ser.read(packet_size)
            if not data:
                continue
            if data == b'T':  # terminatore
                print("Received Terminator Character.")
                break
            f.write(data)
            print(f"📥 Received packet {packet_count+1}")
    ser.close()
    print("📴 Serial COM Port Closed.")

def process_bin_file(bin_filename, csv_filename_prefix=None):
    # Dictionaries to store lists for each data type
    imu_data = {'time': [], 'acc_x': [], 'acc_y': [], 'acc_z': [], 'gyro_x': [], 'gyro_y': [], 'gyro_z': []}
    ppg_data = {'time': [], 'ppg_filtered_ir': []}
    hr_data = {'time': [], 'hr_bpm': []}
    slow_data = {'time': [], 'spo2': [], 'sdnn': [], 'rmssd': [], 'rr': [], 'temp': []}

    DATA_TYPE_PPG = 0
    DATA_TYPE_HR = 1
    DATA_TYPE_IMU_COMBINED = 2
    DATA_TYPE_SLOW_VITALS = 3

    with open(bin_filename, "rb") as f:
        while True:
            pagina = f.read(4096)
            if len(pagina) < 4096:
                break
            
            idx = 0
            while idx < 4096:
                data_type = pagina[idx]
                if data_type == 0xFF:
                    break # End of valid data in this page
                
                idx += 1
                if idx + 5 > 4096: break
                
                hh = pagina[idx]
                mm = pagina[idx+1]
                ss = pagina[idx+2]
                sss = pagina[idx+3] | (pagina[idx+4] << 8)
                idx += 5
                
                time_sec = hh * 3600 + mm * 60 + ss + sss / 1000.0
                
                if data_type == DATA_TYPE_PPG:
                    if idx + 4 > 4096: break
                    ppg_val = struct.unpack('<f', pagina[idx:idx+4])[0]
                    idx += 4
                    ppg_data['time'].append(time_sec)
                    ppg_data['ppg_filtered_ir'].append(ppg_val)
                    
                elif data_type == DATA_TYPE_HR:
                    if idx + 4 > 4096: break
                    hr_val = struct.unpack('<f', pagina[idx:idx+4])[0]
                    idx += 4
                    hr_data['time'].append(time_sec)
                    hr_data['hr_bpm'].append(hr_val)
                    
                elif data_type == DATA_TYPE_IMU_COMBINED:
                    if idx + 12 > 4096: break
                    acc_arr = np.frombuffer(pagina[idx:idx+6], dtype=np.uint8).reshape(1,6)
                    acc_x, acc_y, acc_z = conv_imu(acc_arr)
                    gyro_arr = np.frombuffer(pagina[idx+6:idx+12], dtype=np.uint8).reshape(1,6)
                    gx, gy, gz = conv_gyro(gyro_arr)
                    idx += 12
                    
                    imu_data['time'].append(time_sec)
                    imu_data['acc_x'].append(acc_x[0])
                    imu_data['acc_y'].append(acc_y[0])
                    imu_data['acc_z'].append(acc_z[0])
                    imu_data['gyro_x'].append(gx[0])
                    imu_data['gyro_y'].append(gy[0])
                    imu_data['gyro_z'].append(gz[0])
                    
                elif data_type == DATA_TYPE_SLOW_VITALS:
                    if idx + 18 > 4096: break
                    spo2 = struct.unpack('<f', pagina[idx:idx+4])[0]
                    sdnn = struct.unpack('<f', pagina[idx+4:idx+8])[0]
                    rmssd = struct.unpack('<f', pagina[idx+8:idx+12])[0]
                    rr = struct.unpack('<f', pagina[idx+12:idx+16])[0]
                    temp_raw = struct.unpack('<H', pagina[idx+16:idx+18])[0]
                    idx += 18
                    
                    slow_data['time'].append(time_sec)
                    slow_data['spo2'].append(spo2)
                    slow_data['sdnn'].append(sdnn)
                    slow_data['rmssd'].append(rmssd)
                    slow_data['rr'].append(rr)
                    slow_data['temp'].append(temp_raw * 0.00390625)
                else:
                    # Unknown data type, cannot continue parsing this page
                    break

    df_imu = pd.DataFrame(imu_data)
    df_ppg = pd.DataFrame(ppg_data)
    df_hr = pd.DataFrame(hr_data)
    df_slow = pd.DataFrame(slow_data)

    # plot IMU
    if not df_imu.empty:
        plt.figure(figsize=(15,6))
        plt.subplot(2,1,1)
        plt.plot(df_imu['time'], df_imu['acc_x'], label="acc_x")
        plt.plot(df_imu['time'], df_imu['acc_y'], label="acc_y")
        plt.plot(df_imu['time'], df_imu['acc_z'], label="acc_z")
        plt.title("Accelerometer")
        plt.ylabel("g")
        plt.legend()
        plt.grid(True)

        plt.subplot(2,1,2)
        plt.plot(df_imu['time'], df_imu['gyro_x'], label="gyro_x")
        plt.plot(df_imu['time'], df_imu['gyro_y'], label="gyro_y")
        plt.plot(df_imu['time'], df_imu['gyro_z'], label="gyro_z")
        plt.title("Gyroscope")
        plt.xlabel("Time (s)")
        plt.ylabel("deg/s")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    # plot PPG
    if not df_ppg.empty:
        plt.figure(figsize=(15,4))
        plt.plot(df_ppg['time'], df_ppg['ppg_filtered_ir'], label="ppg_filtered_ir", color="tab:orange")
        plt.title("PPG Filtered IR Signal")
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    # plot Vitals
    if not df_hr.empty or not df_slow.empty:
        fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(15,10), sharex=True)
        
        if not df_hr.empty:
            axs[0].plot(df_hr['time'], df_hr['hr_bpm'], 'ro-', label="HR (bpm)")
            axs[0].set_title("Heart Rate")
            axs[0].set_ylabel("bpm")
            axs[0].legend()
            axs[0].grid(True)
            
        if not df_slow.empty:
            axs[1].plot(df_slow['time'], df_slow['spo2'], 'bo-', label="SpO2 (%)")
            axs[1].set_title("SpO2")
            axs[1].set_ylabel("%")
            axs[1].legend()
            axs[1].grid(True)
            
            axs[2].plot(df_slow['time'], df_slow['rr'], 'go-', label="RR (brpm)")
            axs[2].set_title("Respiration Rate")
            axs[2].set_ylabel("brpm")
            axs[2].legend()
            axs[2].grid(True)
            
            axs[3].plot(df_slow['time'], df_slow['temp'], 'mo-', label="Temperature (°C)")
            axs[3].set_title("Skin Temperature")
            axs[3].set_ylabel("°C")
            axs[3].legend()
            axs[3].grid(True)
            
        plt.xlabel("Time (s)")
        plt.tight_layout()
        plt.show()

    # salva CSVs
    if csv_filename_prefix:
        if not df_imu.empty: df_imu.to_csv(csv_filename_prefix.replace(".csv", "_imu.csv"), index=False)
        if not df_ppg.empty: df_ppg.to_csv(csv_filename_prefix.replace(".csv", "_ppg.csv"), index=False)
        if not df_hr.empty: df_hr.to_csv(csv_filename_prefix.replace(".csv", "_hr.csv"), index=False)
        if not df_slow.empty: df_slow.to_csv(csv_filename_prefix.replace(".csv", "_slow_vitals.csv"), index=False)
        print(f"📄 Data saved as CSVs with prefix: {csv_filename_prefix}")

def gui_select_com_and_folder():
    """Apre una piccola GUI per selezionare COM e cartella."""
    root = Tk()
    root.title("Wearable Monitor - Configuration")
    root.geometry("400x400")
    root.resizable(False, False)

    Label(root, text="🔌 Select the COM Port:", font=("Segoe UI", 10)).pack(pady=5)
    
    com_var = StringVar()
    ports = [p.device for p in list_ports.comports()]

    if not ports:
        ports = ["No COM Port found"]
    com_box = ttk.Combobox(root, textvariable=com_var, values=ports, state="readonly", width=30)
    com_box.pack(pady=5)
    com_box.current(0)

    def browse_folder():
        folder = filedialog.askdirectory(title="📂 Select the folder")
        if folder:
            folder_var.set(folder)

    folder_var = StringVar()
    Label(root, text="📁 Folder:", font=("Segoe UI", 10)).pack(pady=5)
    Button(root, text="Select folder...", command=browse_folder).pack()
    Label(root, textvariable=folder_var, fg="blue", wraplength=350).pack(pady=5)

    def confirm():
        if not folder_var.get() or "Nessuna" in com_var.get():
            messagebox.showerror("Error", "Select a valid COM Port and a folder.")
            return
        root.destroy()

    Button(root, text="✅ Confirm", command=confirm, bg="#4CAF50", fg="white").pack(pady=10)
    root.mainloop()

    return com_var.get(), folder_var.get()

# ==============================
# Main
# ==============================

def main():
    com_port, save_path = gui_select_com_and_folder()
    if not com_port or not save_path:
        print("❌ Application Stopped.")
        return

    base_filename = datetime.now().strftime("WearableData_%Y%m%d_%H%M%S")
    bin_filename = os.path.join(save_path, f"{base_filename}.bin")
    csv_filename = os.path.join(save_path, f"{base_filename}_imu.csv")

    BAUD_RATE = 115200

    try:
        ser = serial.Serial()
        ser.port = com_port
        ser.baudrate = BAUD_RATE
        ser.timeout = 10
        ser.dtr = False
        ser.rts = False
        ser.open()
        print(f"🔌 Connected to {com_port}")
        receive_and_save_data(ser, bin_filename)
    except serial.SerialException as e:
        print(f"⚠️ Serial Error: {e}")
        return

    process_bin_file(bin_filename, csv_filename)

# ==============================
# Entry point
# ==============================

if __name__ == "__main__":
    main()