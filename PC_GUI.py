import tkinter as tk
from tkinter import ttk
from threading import Thread
import serial
from datetime import datetime
from PIL import Image, ImageTk
try:
    from tkintermapview import TkinterMapView
except ImportError:
    print("Warning: tkintermapview library not found. Using fallback map functionality.")
    TkinterMapView = None  # Placeholder for missing library

class DroneSetupGUI:
    def _init_(self, master):
        self.master = master
        self.master.title("Drone GUI Setup")


        # Initialize list to store last 10 positions with timestamps
        #self.last_positions_with_timestamps = []

        # Start a timer to update positions every 1 second
        self.master.after(2000, self.update_positions)

        # Map widget (if tkintermapview is available)
        if TkinterMapView:
            self.map_widget = TkinterMapView(master, width=1000, height=700, corner_radius=10)
            self.map_widget.place(relx=0.6, rely=0.5, anchor=tk.CENTER)
            self.map_widget.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")  # Set tile server (e.g., OpenStreetMap)            #self.map_widget.grid(row=4, column=5, rowspan=10, padx=5, pady=5)
            self.marker = self.map_widget.set_position(1, 1, marker=True)
        else:
            self.map_label = tk.Label(master, text="Map (tkintermapview not found)", width=20, height=10)
            self.map_label.grid(row=0, column=3, rowspan=10, padx=5, pady=5)



        # Load and resize the background image
        img_path = "seecs.png"  # Replace "background_image.png" with your image file path
        self.bg_img = Image.open(img_path)
        self.bg_img = self.bg_img.resize((100, 100), Image.LANCZOS)

        # Apply transparency to the background image
        self.bg_img = self.bg_img.convert("RGBA")
        self.bg_img_with_opacity = Image.new("RGBA", self.bg_img.size, (0, 0, 0, 0))
        self.bg_img = Image.alpha_composite(self.bg_img_with_opacity, self.bg_img)

        # Convert the image to a Tkinter PhotoImage
        self.bg_img_tk = ImageTk.PhotoImage(self.bg_img)

        # Create a label to hold the background image
        self.bg_label = tk.Label(master, image=self.bg_img_tk)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        # Adjust the position of the background image
        self.bg_label.place(relx=1, rely=1, anchor="se", x=120, y=400)

        # Convert your image to .gif format
        img = tk.PhotoImage(file="ensa.gif")  # Replace "your_image.gif" with the path to your .gif file
        self.master.iconphoto(True, img)

        # Data types (modify labels as needed)
        self.angles = ["Pitch", "Yaw", "Roll", "Altitude"]

        # Create dictionaries to store entry fields (easier access during send_data)
        self.gain_entries = {
            "inner": {"Pitch": {}, "Yaw": {}, "Roll": {}, "Altitude": {}},
            "outer": {"Pitch": {}, "Yaw": {}, "Roll": {}, "Altitude": {}},
        }

        # Create labels, entry fields, and buttons for each angle and gain type
        for row_num, angle in enumerate(self.angles):
            if angle == "Yaw":
                inner_p_label = ttk.Label(master, text=f"{angle} Heading P:", style="Custom.TLabel")
                inner_i_label = ttk.Label(master, text=f"{angle} Heading I:", style="Custom.TLabel")
                inner_d_label = ttk.Label(master, text=f"{angle} Heading D:", style="Custom.TLabel")
                outer_p_label = ttk.Label(master, text=f"{angle} Rate P:", style="Custom.TLabel")
                outer_i_label = ttk.Label(master, text=f"{angle} Rate I:", style="Custom.TLabel")
                outer_d_label = ttk.Label(master, text=f"{angle} Rate D:", style="Custom.TLabel")
            else:
                inner_p_label = ttk.Label(master, text=f"{angle} Inner P:", style="Custom.TLabel")
                inner_i_label = ttk.Label(master, text=f"{angle} Inner I:", style="Custom.TLabel")
                inner_d_label = ttk.Label(master, text=f"{angle} Inner D:", style="Custom.TLabel")
                outer_p_label = ttk.Label(master, text=f"{angle} Outer P:", style="Custom.TLabel")
                outer_i_label = ttk.Label(master, text=f"{angle} Outer I:", style="Custom.TLabel")
                outer_d_label = ttk.Label(master, text=f"{angle} Outer D:", style="Custom.TLabel")

            self.gain_entries["inner"][angle]["p"] = ttk.Entry(master)
            self.gain_entries["inner"][angle]["i"] = ttk.Entry(master)
            self.gain_entries["inner"][angle]["d"] = ttk.Entry(master)
            self.gain_entries["outer"][angle]["p"] = ttk.Entry(master)
            self.gain_entries["outer"][angle]["i"] = ttk.Entry(master)
            self.gain_entries["outer"][angle]["d"] = ttk.Entry(master)

            # Use a lambda with angle to capture the current angle
            send_button = ttk.Button(master, text=f"Send {angle} Gains", command=lambda angle=angle: self.send_data(angle))

            inner_p_label.grid(row=row_num*7, column=0, sticky="W", padx=5, pady=5)
            inner_i_label.grid(row=row_num*7+1, column=0, sticky="W", padx=5, pady=5)
            inner_d_label.grid(row=row_num*7+2, column=0, sticky="W", padx=5, pady=5)
            outer_p_label.grid(row=row_num*7+3, column=0, sticky="W", padx=5, pady=5)
            outer_i_label.grid(row=row_num*7+4, column=0, sticky="W", padx=5, pady=5)
            outer_d_label.grid(row=row_num*7+5, column=0, sticky="W", padx=5, pady=5)

            self.gain_entries["inner"][angle]["p"].grid(row=row_num*7, column=1, padx=5, pady=5)
            self.gain_entries["inner"][angle]["i"].grid(row=row_num*7+1, column=1, padx=5, pady=5)
            self.gain_entries["inner"][angle]["d"].grid(row=row_num*7+2, column=1, padx=5, pady=5)
            self.gain_entries["outer"][angle]["p"].grid(row=row_num*7+3, column=1, padx=5, pady=5)
            self.gain_entries["outer"][angle]["i"].grid(row=row_num*7+4, column=1, padx=5, pady=5)
            self.gain_entries["outer"][angle]["d"].grid(row=row_num*7+5, column=1, padx=5, pady=5)

            send_button.grid(row=row_num*7+6, columnspan=2, padx=5, pady=5)

        # Labels for battery and altitude
        self.battery_label = ttk.Label(master, text="Battery (V):", style="Custom.TLabel")
        self.altitude_label = ttk.Label(master, text="Altitude (M):", style="Custom.TLabel")

        # Place labels for battery and altitude
        self.battery_label.grid(row=0, column=2, sticky="W", padx=5, pady=5)
        self.altitude_label.grid(row=1, column=2, sticky="W", padx=5, pady=5)

        # Set up custom style for labels and buttons
        self.master.tk_setPalette(background='#f0f0f0', foreground='#333333')  # Change background and foreground colors
        self.master.option_add('*TLabel.Font', ('Arial', 10))  # Change font for labels
        self.master.option_add('*TButton.Background', '#4CAF50')  # Change button background color
        self.master.option_add('*TButton.Foreground', '#FFFFFF')  # Change button foreground color
        self.master.option_add('*TButton.Font', ('Arial', 10, 'bold'))  # Change button font

        self.master.style = ttk.Style()  # Create a custom style
        self.master.style.configure('Custom.TLabel', background='#f0f0f0', foreground='#333333')  # Configure label style
        self.master.style.configure('Custom.TEntry', background='#FFFFFF', foreground='#333333', bordercolor='#CCCCCC')  # Configure entry style
        self.master.style.map('Custom.TEntry', fieldbackground=[('readonly', '#F0F0F0')])  # Change readonly entry background color
        self.master.style.configure('Custom.TButton', background='#4CAF50', foreground='#FFFFFF', font=('Arial', 10, 'bold'))  # Configure button style

        # UART configuration (replace with your port and baud rate)
        self.ser = serial.Serial(port='COM15', baudrate=115200)  # Change port and baud rate as needed

        # Start a thread to continuously receive data
        self.receive_thread = Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # Load and display the image
        img_path = "ensa.PNG"  # Replace "your_image.png" with the path to your image file
        self.img = Image.open(img_path)
        self.img = self.img.resize((100, 100), Image.LANCZOS)
        self.img = ImageTk.PhotoImage(self.img)
        self.img_label = tk.Label(master, image=self.img)
        self.img_label.place(x=1000, y=795)
        #self.img_label.grid(row=len(self.angles) * 7, column=9, padx=5, pady=5, sticky="se")


    def send_data(self, angle):
        # Get inner and outer gain values from entry fields for the specified angle
        inner_p = self.gain_entries["inner"][angle]["p"].get().strip()
        inner_i = self.gain_entries["inner"][angle]["i"].get().strip()
        inner_d = self.gain_entries["inner"][angle]["d"].get().strip()
        outer_p = self.gain_entries["outer"][angle]["p"].get().strip()
        outer_i = self.gain_entries["outer"][angle]["i"].get().strip()
        outer_d = self.gain_entries["outer"][angle]["d"].get().strip()

        # Define dictionary to map angles to ID numbers
        angle_id_map = {
            "Pitch": 0x01,
            "Yaw": 0x02,
            "Roll": 0x03,
            "Altitude": 0x04,
        }

        Device_id = "0001"

        # Format values with leading zeros (3 digits)
        angle_id_formatted = f"{angle_id_map[angle]:04d}"
        inner_p_formatted = f"{int(inner_p):04d}"
        inner_i_formatted = f"{int(inner_i):04d}"
        inner_d_formatted = f"{int(inner_d):04d}"
        outer_p_formatted = f"{int(outer_p):04d}"
        outer_i_formatted = f"{int(outer_i):04d}"
        outer_d_formatted = f"{int(outer_d):04d}"

        # Construct data string based on angle ID and retrieved values
        data_to_send = f"{Device_id},{angle_id_formatted},{inner_p_formatted},{inner_i_formatted},{inner_d_formatted},{outer_p_formatted},{outer_i_formatted},{outer_d_formatted}".encode()

        # Send data over UART (if configured)
        if self.ser.is_open:
            try:
                self.ser.write(data_to_send)
                print(f"Sending gains for {angle}:")
                print(f"- Angle id: {angle_id_formatted}")
                print(f"- Inner P: {inner_p_formatted}")
                print(f"- Inner I: {inner_i_formatted}")
                print(f"- Inner D: {inner_d_formatted}")
                print(f"- Outer P: {outer_p_formatted}")
                print(f"- Outer I: {outer_i_formatted}")
                print(f"- Outer D: {outer_d_formatted}")
            except serial.SerialException as e:
                print(f"Error sending data: {e}")

    def update_labels(self, battery, altitude, latitude, longitude):
        print(f"- battery: {battery}")
        print(f"- altitude: {altitude}")
        print(f"- latitude: {latitude}")
        print(f"- longitude: {longitude}")
        self.battery_label.config(text=f"Battery (v): {float(battery)/100}")
        self.altitude_label.config(text=f"Altitude (m): {float(altitude)/100}")
        if TkinterMapView:
            # Update map view with received coordinates
            #self.map_widget.set_center((float(latitude)/10000, float(longitude)/10000))
            self.marker.set_position(float(latitude)/10000, float(longitude)/10000)
            # Add current position with timestamp to the list of last 10 positions
            #timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            #self.last_positions_with_timestamps.append(((float(latitude), float(longitude)), timestamp))
        else:
            # Fallback - Update label with coordinates (optional)
            self.map_label.config(text=f"Lat: {float(latitude)/10000}\nLon: {float(longitude)/10000}")

        # If there are more than 10 positions, remove the oldest one
        #if len(self.last_positions_with_timestamps) > 10:
         #   self.last_positions_with_timestamps.pop(0)

        # Draw the path of the last 10 positions on the map
        #if TkinterMapView:
          #  self.map_widget.set_path([pos[0] for pos in self.last_positions_with_timestamps],)

    def update_positions(self):
        # Call the receive_data method to get updated position
        #self.receive_data()

        # Update positions every 1 second
        self.master.after(2000, self.update_positions)

    def receive_data(self):
        # Receive data from UART
        while True:
            if self.ser.is_open:
                try:
                    received_data = self.ser.read(46).decode().strip().split(',')  # Set timeout to avoid blocking
                    if len(received_data) == 8 and received_data[0] == "0003":
                        print(received_data)
                        ,,,,battery, altitude, latitude, longitude = received_data
                        print(f"- battery: {float(battery)/100}")
                        print(f"- altitude: {float(altitude)/100}")
                        print(f"- latitude: {float(latitude)/10000}")
                        print(f"- longitude: {float(longitude)/10000}")
                        self.master.after(0, lambda: self.update_labels(battery, altitude, latitude, longitude))
                except serial.SerialException as e:
                    print(f"Error receiving data: {e}")

# Create the main window
root = tk.Tk()
root.geometry("1500x900")
gui = DroneSetupGUI(root)
'''
map_widget = tkintermapview.TkinterMapView(root, width=1000, height=700, corner_radius=0)
map_widget.place(relx=0.6, rely=0.5, anchor=tk.CENTER)
map_widget.set_position(48.860381, 2.338594,marker=True)
map_widget.set_zoom(15)
'''
if TkinterMapView:
    gui.map_widget.lift()
root.mainloop()