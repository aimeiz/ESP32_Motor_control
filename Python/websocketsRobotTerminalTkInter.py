import json
import asyncio
import websockets
import tkinter as tk
from tkinter import ttk
import threading
print(f"Active threads: {threading.enumerate()}")
import time

class WebSocketsClient:
    def __init__(self):
        self.uri = None
        self.websocket = None
        self.running = False

    async def connect(self, uri):
        """Nawiązanie połączenia WebSocket."""
        try:
            self.uri = uri
            self.websocket = await websockets.connect(uri)
            self.running = True
            print(f"Connected to {uri}")
        except Exception as e:
            print(f"Failed to connect: {e}")

    async def disconnect(self):
        """Zakończenie połączenia WebSocket."""
        self.running = False
        if self.websocket:
            await self.websocket.close()
            print("Disconnected from WebSocket")

    async def send(self, message):
        """Wysłanie wiadomości przez WebSocket."""
        if self.websocket and self.running:
            try:
                await self.websocket.send(message)
                print(f"Sent message: {message}")
            except Exception as e:
                print(f"Error sending message: {e}")

    async def receive(self, callback):
        """Odbieranie wiadomości i wywołanie funkcji callback."""
        print("Receive function started.")  # Debug: Start of the function
        while self.running:
            try:
                message = await self.websocket.recv()
                # print(f"Message received: {message}")
                callback(message)
            except websockets.ConnectionClosed as e:
                print(f"WebSocket connection closed: {e}")
                break
            except Exception as e:
                print(f"Error while receiving: {e}")
                break
        await self.disconnect()


class WebSocketsGUI:
    def __init__(self, master):
        self.master = master # Main window
        self.geometry="460x700"
        self.master.title("WebSocket Robot Terminal")
        self.master.geometry(self.geometry)
        master.config(bg="#121212")  # Background color

        self.websocket_client = WebSocketsClient()
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_event_loop, daemon=True).start()

        #Close connection on application close
        self.master.protocol("WM_DELETE_WINDOW", self.close_application)

        # master.overrideredirect(True)  # Defaolt border off
        # master.wm_attributes("-alpha", 0.97)  # Optional Transparrency

        self.last_speed_update = 0
        self.last_turn_update = 0
        self.update_interval = 0.1  # Minimum interval between slider updates

        # Main Container
        self.container = tk.Frame(self.master, bg="#121212", highlightbackground="#00FF00",  highlightcolor="#00FF00",highlightthickness=2, width=440, height=800)
        self.container.pack_propagate(False)
        self.container.pack(fill="both", expand=True, padx=5, pady=5)
        # Configure consistent column weights for alignment
        for col in range(7):  # Adjust the range based on your columns
            self.container.columnconfigure(col, weight=1)
        # Configure grid weights
        self.container.rowconfigure(list(range(14,24)), weight=1)  # Configure rows for alignment

        #Title Section
        tk.Label(self.container, text="SYSTEM STATUS TABLE", font=("Monospace", 18, "bold"), fg="orange", bg="#121212").grid(
            row=0, column=0, columnspan=8,padx=10, pady=10, sticky="nsew")

        # # # System Table container
        self.systemTableContainer = tk.Frame(self.container, bg="#121212", highlightbackground="#00FF00", highlightthickness=1).grid(
            row=1, column=0, columnspan=8, rowspan=1, padx=0, pady=0, sticky="nsew")
        # System Row
        tk.Label(self.container, text="System:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=1, column=0, padx=5, pady=5, sticky="w")
        tk.Label(self.container, text="tgtSpeed:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=1, column=1, padx=5, pady=5, sticky="w")
        self.tgtSpeedValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.tgtSpeedValue.grid(row=1, column=2, padx=5, pady=5, sticky="e")
        tk.Label(self.container, text="turn:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=1, column=3, padx=5, pady=5, sticky="w")
        self.turnValue= tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.turnValue.grid(row=1, column=4, padx=5, pady=5, sticky="e")
        tk.Label(self.container, text="Dir:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=1, column=6, padx=5, pady=5, sticky="w")
        self.dirValue = tk.Label(self.container, text="XXX", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.dirValue.grid(row=1, column=7, padx=5, pady=5, sticky="e")

        # Left Motor Table
        # Left Motor Row
        tk.Label(self.container, text="Left", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=2, column=0, padx=5, pady=0, sticky="sw")
        tk.Label(self.container, text="tgtSpeed:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=2, column=1, padx=5, pady=2, sticky="w")
        self.leftTgtSpeedValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftTgtSpeedValue.grid(row=2, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="PWM:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=2, column=3, padx=5, pady=2, sticky="w")
        self.leftPWMValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftPWMValue.grid(row=2, column=4, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Dir:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=2, column=6, padx=5, pady=2, sticky="w")
        self.leftDirValue = tk.Label(self.container, text="XXX", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftDirValue.grid(row=2, column=7, padx=5, pady=2, sticky="e")
       #Left Moto lower row
        tk.Label(self.container, text="Motor", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=3, column=0, padx=5, pady=0, sticky="nw")
        tk.Label(self.container, text="Speed:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=3, column=1, padx=5, pady=2, sticky="w")
        self.leftSpeedValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftSpeedValue.grid(row=3, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Way:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=3, column=3, padx=5, pady=2, sticky="w")
        self.leftWayValue = tk.Label(self.container,text="0000000000000",fg="white",bg="#121212",font=("Monospace", 10),anchor="w")
        self.leftWayValue.grid(row=3, column=4, columnspan=4, padx=5, pady=2, sticky="e")

        # # # Right Motor Table
        self.rightMotorTableContainer = tk.Frame(self.container, bg="#121212", highlightbackground="#00FF00", highlightthickness=1).grid(
            row=4, column=0, columnspan=8, rowspan= 2, padx=0, pady=0, sticky="nsew")
        # Right Motor Row
        tk.Label(self.container, text="Right", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=4, column=0, padx=5, pady=0, sticky="sw")
        tk.Label(self.container, text="tgtSpeed:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=4, column=1, padx=5, pady=2, sticky="w")
        self.rightTgtSpeedValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightTgtSpeedValue.grid(row=4, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="PWM:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=4, column=3, padx=5, pady=2, sticky="w")
        self.rightPWMValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightPWMValue.grid(row=4, column=4, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Dir:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=4, column=6, padx=5, pady=2, sticky="w")
        self.rightDirValue = tk.Label(self.container, text="XXX", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightDirValue.grid(row=4, column=7, padx=5, pady=2, sticky="e")
       #Right Moto lower row
        tk.Label(self.container, text="Motor", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=5, column=0, padx=5, pady=0, sticky="nw")
        tk.Label(self.container, text="Speed:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=5, column=1, padx=5, pady=2, sticky="w")
        self.rightSpeedValue = tk.Label(self.container, text="000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightSpeedValue.grid(row=5, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Way:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=5, column=3, padx=5, pady=2, sticky="w")
        # Label for long number
        self.rightWayValue = tk.Label(
            self.container,
            text="0000000000000",  # Initial Text
            fg="white",
            bg="#121212",
            font=("Monospace", 10),
            anchor="w"  # Justify left
        )
        self.rightWayValue.grid(
            row=5,         # Label's row
            column=4,      # Label's column
            columnspan=4,  # Span 4 columns
            padx=5,        # External padding
            pady=2,
            sticky="e"     # Right justify the last column
        )

        #PID Table
        self.PIDonoff=tk.Label(self.container, text="PID: ON", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w")
        self.PIDonoff.grid(row=6, column=0, padx=5, pady=2, sticky="w")
        tk.Label(self.container, text="Kp", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=6, column=1, padx=5, pady=2, sticky="w")
        self.KpValue=tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.KpValue.grid(row=6, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Ki", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=6, column=3, padx=5, pady=2, sticky="w")
        self.KiValue=tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.KiValue.grid(row=6, column=4, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Kd", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=6, column=6, padx=5, pady=2, sticky="w")
        self.KdValue=tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.KdValue.grid(row=6, column=7, padx=5, pady=2, sticky="e")
       
        
        # # PID Left Table
        tk.Label(self.container, text="Left:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=7, column=0, padx=5, pady=2, sticky="w")
        tk.Label(self.container, text="Error:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=7, column=1, padx=5, pady=2, sticky="w")
        self.leftErrorValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftErrorValue.grid(row=7, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Integral:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=7, column=3, padx=5, pady=2, sticky="w")
        self.leftIntegralValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftIntegralValue.grid(row=7, column=4, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Deriv:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=7, column=6, padx=5, pady=2, sticky="w")
        self.leftDerivValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.leftDerivValue.grid(row=7, column=7, padx=5, pady=2, sticky="e")

        # # PID Right Table
        tk.Label(self.container, text="Right:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=8, column=0, padx=5, pady=2, sticky="w")
        tk.Label(self.container, text="Error:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=8, column=1, padx=5, pady=2, sticky="w")
        self.rightErrorValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightErrorValue.grid(row=8, column=2, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Integral::", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=8, column=3, padx=5, pady=2, sticky="w")
        self.rightIntegralValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightIntegralValue.grid(row=8, column=4, padx=5, pady=2, sticky="e")
        tk.Label(self.container, text="Deriv:", fg="cyan", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=8, column=6, padx=5, pady=2, sticky="w")
        self.rightDerivValue = tk.Label(self.container, text="0.0000", fg="white", bg="#121212", font=("Monospace", 10), anchor="w")
        self.rightDerivValue.grid(row=8, column=7, padx=5, pady=2, sticky="e")


        # # Battery Table
        self.batteryTableContainer = tk.Frame(self.container, bg="#121212", highlightbackground="#00FF00", highlightthickness=1).grid(
            row=9, column=0, columnspan=8, padx=0, pady=0, sticky="nsew")
        tk.Label(self.container, text="Battery:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=9, column=0, padx=5, pady=2, sticky="w")
        tk.Label(self.container, text="Voltage:", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="w").grid(
            row=9, column=1, padx=5, pady=2, sticky="w")
        self.batVoltageValue = tk.Label(self.container, text="0.00", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w")
        self.batVoltageValue.grid(row=9, column=2, padx=5, pady=2, sticky="e")
        self.batVoltageVolt = tk.Label(self.container, text="V", fg="#00FF00", bg="#121212", font=("Monospace", 10), anchor="w")
        self.batVoltageVolt.grid(row=9, column=3, padx=5, pady=2, sticky="w")

        # Sliders Section
        self.sliders_frame = tk.Frame(self.container, bg="#121212")
        self.sliders_frame.grid(row=10, column=0, columnspan=8, padx=5, pady=0, sticky="nsew")

        # # Configure grid weights for even distribution
        self.sliders_frame.columnconfigure(0, weight=1)
        self.sliders_frame.columnconfigure(1, weight=1)

        # # Speed Slider
        self.is_updating_speed = False
        tk.Label(self.sliders_frame, text="Speed", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="center").grid(
            row=0, column=0, padx=5, columnspan=1,pady=5, sticky="we")
        self.speed_slider = tk.Scale(self.sliders_frame, from_=0, to=100, orient="horizontal", bg="#121212", fg="#00FF00",highlightbackground="cyan")
        print(f"Speed slider value: {self.speed_slider.get()}")
        self.speed_slider.grid(row=1, column=0, columnspan=1, padx=5, pady=5, sticky="we")
        # Speed Slider bindings
        self.speed_slider.bind("<B1-Motion>", self.update_speed)       # Called during slider movement
        self.speed_slider.bind("<ButtonRelease-1>", self.final_speed)  # Final update when slider is released
        self.speed_slider.bind("<ButtonPress-1>", self.handle_speed_start)  # Track slider interaction start
        self.speed_slider.bind("<ButtonRelease-1>", self.handle_speed_end)  # Track slider interaction end


        # # Turn Slider
        self.is_updating_turn = False
        tk.Label(self.sliders_frame, text="Turn", fg="yellow", bg="#121212", font=("Monospace", 10), anchor="center").grid(
            row=0, column=1, columnspan=1,padx=5, pady=5, sticky="we")
        self.turn_slider = tk.Scale(self.sliders_frame, from_=-100, to=100, orient="horizontal", bg="#121212", fg="#00FF00",highlightbackground="cyan")
        print(f"Speed slider value: {self.speed_slider.get()}")
        self.turn_slider.grid(row=1, column=1, columnspan=1, padx=5, pady=5, sticky="we")
        # Turn Slider bindings
        self.turn_slider.bind("<B1-Motion>", self.update_turn)        # Called during slider movement
        self.turn_slider.bind("<ButtonRelease-1>", self.final_turn)  # Final update when slider is released
        self.turn_slider.bind("<ButtonPress-1>", self.handle_turn_start)  # Track slider interaction start
        self.turn_slider.bind("<ButtonRelease-1>", self.handle_turn_end)  # Track slider interaction end

        # Buttons Section
        self.buttons_frame = tk.Frame(self.container, bg="#121212", highlightbackground="#00FF00", highlightthickness=1)
        self.buttons_frame.grid(row=12, column=0, columnspan=8, padx=0, pady=0, sticky="nsew")

        # # Configure grid weights for even button distribution
        self.buttons_frame.columnconfigure(0, weight=1)
        self.buttons_frame.columnconfigure(1, weight=1)
        self.buttons_frame.columnconfigure(2, weight=1)

        #Backward Button with border
        self.createButton(
            name="backward_button",  # Nazwa przycisku w klasie
            container=self.buttons_frame,  # Kontener, w którym przycisk ma się pojawić
            row=0,  # Wiersz w siatce
            column=0,  # Kolumna w siatce
            text="BACKWARD",  # Tekst na przycisku
            command=lambda: self.send_command(command="BACKWARD"),  # Funkcja wywoływana po kliknięciu
            columnSpan=1  # Przycisk zajmuje 1 kolumnę
        )
        #Stop Button with border
        self.createButton(
            name="stop_button",  # Nazwa przycisku w klasie
            container=self.buttons_frame,  # Kontener, w którym przycisk ma się pojawić
            row=0,  # Wiersz w siatce
            column=1,  # Kolumna w siatce
            text="STOP",  # Tekst na przycisku
            command=lambda: self.send_command(command="STOP"),  # Funkcja wywoływana po kliknięciu
            columnSpan=1  # Przycisk zajmuje 1 kolumnę
        )
        #Forward Button with border
        self.createButton(
            name="forward_button",  # Nazwa przycisku w klasie
            container=self.buttons_frame,  # Kontener, w którym przycisk ma się pojawić
            row=0,  # Wiersz w siatce
            column=2,  # Kolumna w siatce
            text="FORWARD",  # Tekst na przycisku
            command=lambda: self.send_command(command="FORWARD"),  # Funkcja wywoływana po kliknięciu
            columnSpan=1  # Przycisk zajmuje 1 kolumnę
        )

#         # # Command Input Section
        self.command_frame = tk.LabelFrame(self.container, text="Enter command", fg="#00FF00", bg="#121212", highlightbackground="#00FF00", highlightthickness=1,  highlightcolor="#00FF00")
        self.command_frame.grid(row=13, column=0, columnspan=7, padx=5, pady=5, sticky="nsew")
          # # Configure grid layout to make the input field expand
        self.command_frame.columnconfigure(0, weight=1)


        # # Command Input Field
        self.command_input=tk.Entry(self.command_frame, font=("Monospace", 12), bg="#121212", fg="yellow")
        self.command_input.grid(row=0, column=0, columnspan=1,padx=0, pady=0, sticky="ew")
        self.command_input.insert(0, "ws://192.168.1.82:83/ws")
        # Zdarzenia
        self.command_input.bind("<Return>", self.send_command)  # Wyślij komendę po Enter

        # # Send Button
        self.createButton(
            name="command_button",  # Nazwa przycisku w klasie
            container=self.container,  # Kontener, w którym przycisk ma się pojawić
            row=13,  # Wiersz w siatce
            column=7,  # Kolumna w siatce
            text="Send",  # Tekst na przycisku
            command=self.send_command,# Funkcja wywoływana po kliknięciu
            fg="cyan"
            # columnSpan=1  # Przycisk zajmuje 1 kolumny
        )
        self.ordersResponse = tk.Label(self.container, text="Response: ", font=("Monospace", 10), fg="white", bg="#121212", anchor="w")
        self.ordersResponse.grid(row=14, column=0, columnspan=8, padx=5, pady=5, sticky="w")        
        # # Menu Section
        commands = [
            "System commands: \t\t\t\t\tHide\r\n"
            "[F]ORWARD <speed> <distance>",
            "[B]ACKWARD <speed> <distance>",
            "[T]URN <turn> - Set turn 0 - +/-100[%]",
            "[S]TOP",
            "[P]ID <kp> <ki> <kd> - Enter PID factors",
            "P[I]DON <0/1> - PID regulation Off / On",
            "[L]OG <Interval[s]> - Log variables to /log.txt",
            "[SIM]MULATION <0/1> - Simulation mode Off / On",
        ]
        self.comandsString=""
        for command in commands:
            self.comandsString+=command+"\n"

        self.hide_button = tk.Button(self.container,text=self.comandsString, anchor="w", justify="left", command=self.toggle_commands, activebackground="#121212", activeforeground="yellow", bg="#121212", fg="yellow", font=("Monospace", 9, "bold"))
        self.hide_button.grid(row=15, column=0, columnspan=8,sticky="w", padx=5, pady=5)

        # # OTA Hostname
        self.ota_label = tk.Label(self.container, text="OTA: -", font=("Monospace", 8), fg="#00FF00", bg="#121212")
        self.ota_label.grid(row=27, column=0, columnspan=8,sticky="w")

        # Start Async Loop
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_event_loop, daemon=True).start()

    def createButton(self, name, container, row, column, text="button", command=None, bg="#121212", fg="yellow", highlightbackground="#00FF00", highlightthickness=1, columnSpan=1):
        """Tworzy przycisk w podanym kontenerze z podaną funkcją przypisaną do akcji."""
        frame = tk.Frame(container, bg=bg, highlightbackground=highlightbackground, highlightthickness=highlightthickness)
        frame.grid(row=row, column=column, columnspan=columnSpan, padx=5, pady=5, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        # Create button inside the frame
        button = tk.Button(
            frame,
            text=text,
            bg="#121212",
            fg=fg,
            relief="flat",
            command=command  # Function assigned to the button
        )
        button.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")

        button.bind("<Enter>", lambda e: button.config(fg=bg, bg=highlightbackground))
        button.bind("<Leave>", lambda e: button.config(fg=fg, bg=bg))

        setattr(self, name, button)  # Save button in the class 
    
    def toggle_commands(self):
        if self.hide_button.cget("text")==self.comandsString:
            self.master.geometry("460x560")
            self.hide_button.config(text="System commands: \t\t\t\t\tShow")
        else:
            self.master.geometry(self.geometry)
            self.hide_button.config(text=self.comandsString)

    def send_command(self, event=None, command=None):
        """Command sending function."""
        if not command:
            command = self.command_input.get().strip()

        # If the command is a WebSocket address
        if command.startswith("ws://"):
            # establish a new connection
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.connect(command), self.loop
            )
            try:
                future.result()  # await connection
                print(f"Connected to {command}")
                self.command_frame.config(text="Connected")
                self.start_receive_loop()  # Start receiving messages after connecting
            except Exception as e:
                print(f"Error connecting to {command}: {e}")
        elif self.websocket_client.running:
            # Send the command to the server
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.send(command), self.loop
            )
            try:
                future.result()
                print(f"Command sent: {command}")
            except Exception as e:
                print(f"Error sending command: {e}")

        # Command input field cleanup
        self.command_input.delete(0, tk.END)

    def start_receive_loop(self):
        """Start the WebSocket receive loop."""
        print("Starting receive loop...")  # Debug
        if self.websocket_client.running:
            asyncio.run_coroutine_threadsafe(
                self.websocket_client.receive(self.update_table), self.loop
            )
    def handle_speed_start(self, event):
        """User starts interacting with the speed slider."""
        self.is_updating_speed = True

    def handle_speed_end(self, event):
        """User stops interacting with the speed slider."""
        self.is_updating_speed = False
        # self.update_speed(self.speed_slider.get())

    def handle_turn_start(self, event):
        """User starts interacting with the turn slider."""
        self.is_updating_turn = True

    def handle_turn_end(self, event):
        """User stops interacting with the turn slider."""
        self.is_updating_turn = False
    #print(f"Error sending speed update: {e}")
    def update_speed(self, event):
        """Send speed updates while the slider is moving."""
        # Cancel any pending updates
        if hasattr(self, "_speed_update_timer"):
            self.master.after_cancel(self._speed_update_timer)

        # Schedule a new update after a short delay
        self._speed_update_timer = self.master.after(100, self._send_speed_update)

    def _send_speed_update(self):
        """Send the speed update command to the server."""
        value = self.speed_slider.get()
        print(f"Updating speed to {value}")  # Debug
        command = f"SPEED {value}"
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.send(command), self.loop
            )
            future.result(timeout=1)
            print(f"Sent message: {command}")
        except Exception as e:
            print(f"Error sending speed update: {e}")

    def final_speed(self, event):
        """Send the final speed command when the slider is released."""
        value = self.speed_slider.get()
        print(f"Final speed set to {value}")  # Debug
        command = f"SPEED {value}"
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.send(command), self.loop
            )
            future.result(timeout=1)
            print(f"Sent message: {command}")
        except Exception as e:
            print(f"Error sending final speed update: {e}")

    def update_turn(self, event):
        """Send turn updates while the slider is moving."""
        # Cancel any pending updates
        if hasattr(self, "_turn_update_timer"):
            self.master.after_cancel(self._turn_update_timer)

        # Schedule a new update after a short delay
        self._turn_update_timer = self.master.after(100, self._send_turn_update)

    def _send_turn_update(self):
        """Send the turn update command to the server."""
        value = self.turn_slider.get()
        print(f"Updating turn to {value}")  # Debug
        command = f"TURN {value}"
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.send(command), self.loop
            )
            future.result(timeout=1)
            print(f"Sent message: {command}")
        except Exception as e:
            print(f"Error sending turn update: {e}")

    def final_turn(self, event):
        """Send the final turn command when the slider is released."""
        value = self.turn_slider.get()
        print(f"Final turn set to {value}")  # Debug
        command = f"TURN {value}"
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.websocket_client.send(command), self.loop
            )
            future.result(timeout=1)
            print(f"Sent message: {command}")
        except Exception as e:
            print(f"Error sending final turn update: {e}")

    def handle_turn_start(self, event):
        """User starts interacting with the turn slider."""
        self.is_updating_turn = True

    def handle_turn_end(self, event):
        """User stops interacting with the turn slider."""
        self.is_updating_turn = False


    def start_event_loop(self):
        print("Event loop started")  # Debug message
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        print("Event loop stopped.")

    def update_table(self, message):
        """GUI table update according to data received."""
        try:
            data = json.loads(message)
            targetSpeedRcv = data.get("targetSpeed", "000")
            self.tgtSpeedValue.config(text=targetSpeedRcv)
            if not self.is_updating_speed:  # Only update if the user isn't interacting
                self.speed_slider.set(value=targetSpeedRcv)
            turnRcv = data.get("turn", "000")        
            self.turnValue.config(text=turnRcv)
            if not self.is_updating_turn:  # Only update if the user isn't interacting
                self.turn_slider.set(value=turnRcv) 

            self.dirValue.config(text="FWD" if data.get("direction", 0) == 0 else "REV")
            self.leftTgtSpeedValue.config(text=data.get("targetLeftSpeed", "000"))
            self.leftPWMValue.config(text=data.get("leftMotorPwm", "000"))
            self.leftDirValue.config(text=data.get("leftMotorDirection", "XXX"))
            self.leftSpeedValue.config(text=data.get("speedLeft", "000"))
            self.leftWayValue.config(text=data.get("leftWay", "0000000"))

            self.rightTgtSpeedValue.config(text=data.get("targetRightSpeed", "000"))
            self.rightPWMValue.config(text=data.get("rightMotorPwm", "000"))
            self.rightDirValue.config(text=data.get("rightMotorDirection", "XXX"))
            self.rightSpeedValue.config(text=data.get("speedRight", "000"))
            self.rightWayValue.config(text=data.get("rightWay", "0000000"))

            self.PIDonoff.config(text="PID: ON" if data.get("PID", 0) == "ON" else "PID: OFF")
            self.KpValue.config(text=f"{data.get('kp', 0.000):.3f}")
            self.KiValue.config(text=f"{data.get('ki', 0.000):.3f}")
            self.KdValue.config(text=f"{data.get('kd', 0.000):.3f}")

            self.leftErrorValue.config(text=f"{data.get('errLeft', 0.000):.3f}")
            self.leftIntegralValue.config(text=f"{data.get('intLeft', 0.000):.3f}")
            self.leftDerivValue.config(text=f"{data.get('derLeft', 0.000):.3f}")

            self.rightErrorValue.config(text=f"{data.get('errRight', 0.000):.3f}")
            self.rightIntegralValue.config(text=f"{data.get('intRight', 0.000):.3f}")
            self.rightDerivValue.config(text=f"{data.get('derRight', 0.000):.3f}")

            self.batVoltageValue.config(text=f"{data.get('batteryVoltage', 0.00):.2f}")
            if data.get('batteryStatus') == "Low":
                color = "red"
            else: color = "#00FF00"
            self.batVoltageValue.config(fg=color)
            self.batVoltageVolt.config(fg=color)
            self.ordersResponse.config(text=(f"Response: {data.get('ordersResponse', ' ')}").strip())
            self.ota_label.config(text=f"{data.get('otaHostname', '-')}")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def close_application(self):
        """Close the application and disconnect the WebSocket."""
        print("Closing application...")
        try:
            if self.websocket_client.running:
                print("Disconnecting WebSocket...")
                future = asyncio.run_coroutine_threadsafe(
                    self.websocket_client.disconnect(), self.loop
                )
                future.result(timeout=5)  # Timeout for disconnection
                print("WebSocket disconnected.")
        except Exception as e:
            print(f"Error during WebSocket disconnection: {e}")

        print("Stopping event loop...")
        self.loop.stop()  # Stop the asyncio loop
        self.master.destroy()  # Destroy the main window
        print("Application closed.")


if __name__ == "__main__":
    root = tk.Tk()
    app = WebSocketsGUI(root)
    root.mainloop()
