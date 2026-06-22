import numpy as np
import matplotlib.pyplot as plt
import serial
import os
import pandas as pd
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

def apply_filter(signal):
    """
    Applies the 2-stage Direct Form I Biquad Cascade filter to the input signal.
    Stage 1: 2nd-order Butterworth LPF (fc = 5.0 Hz, fs = 200.0 Hz)
    Stage 2: 2nd-order Butterworth HPF (fc = 0.5 Hz, fs = 200.0 Hz)
    """
    # Stage 1 (LPF: 5 Hz) coefficients, fs = 200 Hz
    b0_1, b1_1, b2_1 = 0.00554277, 0.01108554, 0.00554277
    a1_1, a2_1 = 1.778631, -0.800802
    
    # Stage 2 (HPF: 0.5 Hz) coefficients, fs = 200 Hz
    b0_2, b1_2, b2_2 = 0.9889543, -1.9779086, 0.9889543
    a1_2, a2_2 = 1.977786, -0.978031
    
    n = len(signal)
    output = np.zeros(n)
    
    # Stage 1 states (Direct Form I: x[n-1], x[n-2], y[n-1], y[n-2])
    s1_x1, s1_x2 = 0.0, 0.0
    s1_y1, s1_y2 = 0.0, 0.0
    
    # Stage 2 states
    s2_x1, s2_x2 = 0.0, 0.0
    s2_y1, s2_y2 = 0.0, 0.0
    
    for i in range(n):
        # Stage 1
        x = float(signal[i])
        y1 = b0_1 * x + b1_1 * s1_x1 + b2_1 * s1_x2 + a1_1 * s1_y1 + a2_1 * s1_y2
        s1_x2 = s1_x1
        s1_x1 = x
        s1_y2 = s1_y1
        s1_y1 = y1
        
        # Stage 2
        y2 = b0_2 * y1 + b1_2 * s2_x1 + b2_2 * s2_x2 + a1_2 * s2_y1 + a2_2 * s2_y2
        s2_x2 = s2_x1
        s2_x1 = y1
        s2_y2 = s2_y1
        s2_y1 = y2
        
        output[i] = y2
        
    return output

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

def process_bin_file(bin_filename, csv_filename=None):
    hh_list, mm_list, ss_list, sss_list = [], [], [], []
    ppg0_list, ppg1_list = [], []

    # Each NAND page is sent as 4096 bytes. We ignore the last 16 spare bytes
    # and parse subpackets. New subpacket layout: 5 bytes timestamp, 3 bytes PPG0,
    # 3 bytes PPG1 -> total 11 bytes per subpacket
    SUBPKT_SIZE = 11
    with open(bin_filename, "rb") as f:
        while True:
            pagina = f.read(4096)
            if len(pagina) < 4096:
                break
            # consider only the first 4080 bytes (4096 - 16 spare bytes)
            valid_bytes = pagina[:4080]
            # parse subpackets of SUBPKT_SIZE bytes
            for i in range(0, len(valid_bytes), SUBPKT_SIZE):
                subpkt = valid_bytes[i:i+SUBPKT_SIZE]
                if len(subpkt) < SUBPKT_SIZE:
                    continue
                # timestamp
                hh = subpkt[0]
                mm = subpkt[1]
                ss = subpkt[2]
                sss = subpkt[3] | (subpkt[4] << 8)
                hh_list.append(hh)
                mm_list.append(mm)
                ss_list.append(ss)
                sss_list.append(sss)
                # PPG (MAX30101): two 24-bit big-endian values placed after
                # the 5-byte timestamp: bytes 5..7 -> ppg0, bytes 8..10 -> ppg1
                ppg0 = int.from_bytes(subpkt[5:8], byteorder='big')
                ppg1 = int.from_bytes(subpkt[8:11], byteorder='big')
                ppg0_list.append(ppg0)
                ppg1_list.append(ppg1)

    # apply bandpass filter
    ppg0_filtered = apply_filter(ppg0_list)
    ppg1_filtered = apply_filter(ppg1_list)

    # crea DataFrame
    df = pd.DataFrame({
        "hh": hh_list,
        "mm": mm_list,
        "ss": ss_list,
        "sss": sss_list,
        "ppg_led0": ppg0_list,
        "ppg_led0_filtered": ppg0_filtered,
        "ppg_led1": ppg1_list,
        "ppg_led1_filtered": ppg1_filtered
    })

    # plot PPG (MAX30101) - Raw vs Filtered
    fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(12, 8))
    
    # Raw signals
    axs[0].plot(df.index, df["ppg_led0"], label="ppg_led0 (raw)", color="tab:blue")
    axs[0].plot(df.index, df["ppg_led1"], label="ppg_led1 (raw)", color="tab:orange")
    axs[0].set_title("MAX30101 PPG - Raw")
    axs[0].set_ylabel("Raw value")
    axs[0].legend()
    axs[0].grid(True)

    # Filtered signals
    axs[1].plot(df.index, df["ppg_led0_filtered"], label="ppg_led0 (filtered)", color="tab:blue")
    axs[1].plot(df.index, df["ppg_led1_filtered"], label="ppg_led1 (filtered)", color="tab:orange")
    axs[1].set_title("MAX30101 PPG - Filtered")
    axs[1].set_xlabel("Subpacket index")
    axs[1].set_ylabel("Filtered value")
    axs[1].legend()
    axs[1].grid(True)

    fig.suptitle("MAX30101 PPG Data Analysis")
    plt.tight_layout()
    plt.show()

    # salva CSV
    if csv_filename:
        df.to_csv(csv_filename, index=False)
        print(f"📄 Data saved in CSV: {csv_filename}")

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

    BAUD_RATE = 250000

    try:
        ser = serial.Serial(com_port, BAUD_RATE, timeout=10)
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