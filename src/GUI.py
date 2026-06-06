import asyncio
import math
import threading
import time
import tkinter as tk
from tkinter import messagebox

from bleak import BleakClient, BleakScanner


NOM_ROBOT = "DRAWBOT"

CHAR_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHAR_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class DrawbotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("DRAWBOT Control Center")
        self.root.geometry("1200x760")
        self.root.minsize(980, 640)
        self.root.configure(bg="#0F172A")
        self.root.bind("<Escape>", lambda event: self.root.attributes("-fullscreen", False))
        self.root.protocol("WM_DELETE_WINDOW", self.quitter)

        self.client = None
        self.is_connecting = False
        self.last_imu_time = None
        self.advanced_sequences_visible = False
        self.sim_after_id = None
        self.sim_points = []
        self.sim_target_points = []
        self.sim_fixed_bounds = None
        self.sim_drawn_until = 0
        self.sim_title = "Aucune simulation"

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0

        self.calibration_count = 0
        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.gz_offset = 0.0

        self.telemetry_values = {
            "accel": "--- / --- / ---",
            "gyro": "--- / --- / ---",
            "mag": "--- / --- / ---",
            "speed_l": "--- ticks/s",
            "speed_r": "--- ticks/s",
            "ticks": "G=--- / D=---",
            "distance_robot": "--- cm",
            "position": "X=--- cm / Y=--- cm",
            "orientation": "G/D=--- deg / A/R=--- deg / Direction=--- deg",
        }

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()

        self.colors = {
            "bg": "#0F172A",
            "panel": "#111827",
            "panel_2": "#172033",
            "border": "#273449",
            "text": "#E5E7EB",
            "muted": "#94A3B8",
            "accent": "#38BDF8",
            "accent_dark": "#0284C7",
            "green": "#22C55E",
            "red": "#EF4444",
            "orange": "#F59E0B",
            "button": "#1E293B",
            "button_active": "#334155",
            "input": "#020617",
        }

        # Fix macOS Sequoia : forcer les couleurs personnalisées
        try:
            self.root.tk.call("set", "::tk::mac::useThemedToplevel", "0")
        except Exception:
            pass
        self.root.tk_setPalette(
            background=self.colors["bg"],
            foreground=self.colors["text"],
            activeBackground=self.colors["button_active"],
            activeForeground=self.colors["text"],
            highlightBackground=self.colors["bg"],
            highlightColor=self.colors["border"],
            selectBackground=self.colors["accent"],
            selectForeground="#031525",
        )

        self.build_interface()
        self.refresh_status("Déconnecté", self.colors["red"])
        self.refresh_telemetry()

    def build_interface(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = tk.Frame(self.root, bg=self.colors["bg"], padx=28, pady=22, highlightthickness=0)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        tk.Label(
            header,
            text="DRAWBOT CONTROL CENTER",
            bg=self.colors["bg"],
            fg=self.colors["text"],
            font=("Arial", 24, "bold"),
        ).grid(row=0, column=0, sticky="w")

        tk.Label(
            header,
            text="Pilotage BLE, mouvement précis et télémétrie embarquée",
            bg=self.colors["bg"],
            fg=self.colors["muted"],
            font=("Arial", 12),
        ).grid(row=1, column=0, sticky="w", pady=(4, 0))

        status_box = tk.Frame(header, bg=self.colors["panel"], padx=14, pady=10, highlightthickness=0)
        status_box.grid(row=0, column=1, rowspan=2, sticky="e")

        self.status_dot = tk.Canvas(status_box, width=13, height=13, bg=self.colors["panel"], highlightthickness=0)
        self.status_dot.grid(row=0, column=0, padx=(0, 8))
        self.status_dot_id = self.status_dot.create_oval(2, 2, 11, 11, fill=self.colors["red"], outline="")

        self.status = tk.Label(
            status_box,
            text="Déconnecté",
            fg=self.colors["red"],
            bg=self.colors["panel"],
            font=("Arial", 12, "bold"),
        )
        self.status.grid(row=0, column=1, sticky="w")

        scroll_area = tk.Frame(self.root, bg=self.colors["bg"], highlightthickness=0)
        scroll_area.grid(row=1, column=0, sticky="nsew")
        scroll_area.columnconfigure(0, weight=1)
        scroll_area.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(scroll_area, bg=self.colors["bg"], highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        scrollbar = tk.Scrollbar(scroll_area, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        content = tk.Frame(self.canvas, bg=self.colors["bg"], padx=34, pady=10, highlightthickness=0)
        content.columnconfigure(0, weight=1)
        self.content_window = self.canvas.create_window((0, 0), window=content, anchor="nw")

        content.bind("<Configure>", self.update_scroll_region)
        self.canvas.bind("<Configure>", self.resize_scroll_content)
        self.root.bind_all("<MouseWheel>", self.on_mousewheel)
        self.root.bind_all("<Button-4>", self.on_mousewheel)
        self.root.bind_all("<Button-5>", self.on_mousewheel)

        self.connection_panel(content).grid(row=0, column=0, sticky="ew", pady=(0, 26))
        self.manual_panel(content).grid(row=1, column=0, sticky="ew", pady=(0, 26))
        self.distance_panel(content).grid(row=2, column=0, sticky="ew", pady=(0, 26))
        self.sequence_panel(content).grid(row=3, column=0, sticky="ew", pady=(0, 26))
        self.simulation_panel(content).grid(row=4, column=0, sticky="ew", pady=(0, 26))
        self.telemetry_panel(content).grid(row=5, column=0, sticky="ew", pady=(0, 38))

    def update_scroll_region(self, event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def resize_scroll_content(self, event):
        self.canvas.itemconfig(self.content_window, width=event.width)

    def on_mousewheel(self, event):
        if getattr(event, "num", None) == 4:
            self.canvas.yview_scroll(-3, "units")
            return

        if getattr(event, "num", None) == 5:
            self.canvas.yview_scroll(3, "units")
            return

        if event.delta == 0:
            return

        direction = -1 if event.delta > 0 else 1
        amount = max(1, min(8, abs(event.delta) // 30))
        self.canvas.yview_scroll(direction * amount, "units")

    def panel(self, parent, title, subtitle=None, number=None):
        frame = tk.Frame(parent, bg=self.colors["panel"], padx=26, pady=24, highlightthickness=0)
        frame.columnconfigure(0, weight=1)

        title_row = tk.Frame(frame, bg=self.colors["panel"], highlightthickness=0)
        title_row.grid(row=0, column=0, sticky="ew")
        title_row.columnconfigure(1, weight=1)

        if number:
            tk.Label(
                title_row,
                text=number,
                bg=self.colors["accent"],
                fg="#031525",
                font=("Arial", 11, "bold"),
                padx=10,
                pady=4,
            ).grid(row=0, column=0, sticky="w", padx=(0, 12))

        tk.Label(
            title_row,
            text=title,
            bg=self.colors["panel"],
            fg=self.colors["text"],
            font=("Arial", 21, "bold"),
        ).grid(row=0, column=1, sticky="w")

        if subtitle:
            tk.Label(
                frame,
                text=subtitle,
                bg=self.colors["panel"],
                fg=self.colors["muted"],
                font=("Arial", 11),
                wraplength=820,
                justify="left",
            ).grid(row=1, column=0, sticky="w", pady=(4, 18))

        return frame

    def make_button(self, parent, text, command, kind="default", width=16):
        bg = self.colors["button"]
        fg = "#111827"
        active = self.colors["button_active"]

        if kind == "primary":
            bg = self.colors["accent"]
            fg = "#031525"
            active = self.colors["accent_dark"]
        elif kind == "danger":
            bg = self.colors["red"]
            fg = "#111827"
            active = "#B91C1C"
        elif kind == "success":
            bg = self.colors["green"]
            fg = "#04130A"
            active = "#15803D"

        return tk.Button(
            parent,
            text=text,
            command=command,
            font=("Arial", 11, "bold"),
            bg=bg,
            fg=fg,
            activebackground=active,
            activeforeground=fg,
            disabledforeground="#111827",
            relief="flat",
            bd=0,
            width=width,
            height=2,
            cursor="hand2",
        )

    def connection_panel(self, parent):
        frame = self.panel(parent, "Connexion Bluetooth", "Première étape: connecter l'interface au robot DRAWBOT.", "01")
        frame.columnconfigure((0, 1), weight=1)

        self.connect_btn = self.make_button(frame, "Connecter", self.connecter, "primary", width=20)
        self.connect_btn.grid(row=2, column=0, sticky="ew", padx=(0, 8))

        self.disconnect_btn = self.make_button(frame, "Déconnecter", self.deconnecter, "default", width=20)
        self.disconnect_btn.grid(row=2, column=1, sticky="ew", padx=(8, 0))

        tk.Label(
            frame,
            text="Les commandes envoyées restent identiques à l'ancienne interface.",
            bg=self.colors["panel"],
            fg=self.colors["muted"],
            font=("Arial", 10),
            wraplength=820,
            justify="left",
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(16, 0))

        self.make_button(frame, "Quitter", self.quitter, "danger", width=20).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(18, 0)
        )

        return frame

    def manual_panel(self, parent):
        frame = self.panel(parent, "Pilotage manuel", "Commandes directes du robot: avancer, reculer, tourner à gauche, tourner à droite et arrêt immédiat.", "02")
        frame.columnconfigure(0, weight=1)

        pad = {"padx": 8, "pady": 8}
        manual = tk.Frame(frame, bg=self.colors["panel"], highlightthickness=0)
        manual.grid(row=2, column=0, sticky="ew")
        manual.columnconfigure((0, 1, 2), weight=1)

        self.make_button(manual, "Avancer", lambda: self.envoyer("A")).grid(row=0, column=1, sticky="ew", **pad)
        self.make_button(manual, "Gauche", lambda: self.envoyer("G")).grid(row=1, column=0, sticky="ew", **pad)
        self.make_button(manual, "STOP", lambda: self.envoyer("S"), "danger").grid(row=1, column=1, sticky="ew", **pad)
        self.make_button(manual, "Droite", lambda: self.envoyer("D")).grid(row=1, column=2, sticky="ew", **pad)
        self.make_button(manual, "Reculer", lambda: self.envoyer("R")).grid(row=2, column=1, sticky="ew", **pad)

        return frame

    def distance_panel(self, parent):
        frame = self.panel(parent, "Déplacements précis", "Avancer ou reculer d'une distance définie, puis tourner selon un angle choisi.", "03")
        frame.columnconfigure((0, 1), weight=1)

        pad = {"padx": 8, "pady": 8}
        precise = tk.Frame(frame, bg=self.colors["panel"])
        precise.grid(row=2, column=0, columnspan=2, sticky="ew")
        precise.columnconfigure((0, 1, 2, 3), weight=1)

        tk.Label(precise, text="Distance (cm)", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=8
        )
        self.distance = self.make_entry(precise, "20")
        self.distance.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 12))

        self.make_button(precise, "Avancer cm", self.avancer_cm, "success").grid(row=2, column=0, sticky="ew", **pad)
        self.make_button(precise, "Reculer cm", self.reculer_cm).grid(row=2, column=1, sticky="ew", **pad)

        tk.Label(precise, text="Angle (degrés)", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=2, columnspan=2, sticky="w", padx=8
        )
        self.angle = self.make_entry(precise, "90")
        self.angle.grid(row=1, column=2, columnspan=2, sticky="ew", padx=8, pady=(4, 12))

        self.make_button(precise, "Tourner angle", self.tourner_angle, "primary").grid(
            row=2, column=2, columnspan=2, sticky="ew", **pad
        )

        return frame

    def sequence_panel(self, parent):
        frame = self.panel(parent, "Séquences", "Choisis une séquence du sujet puis lance le robot.", "04")
        frame.columnconfigure((0, 1), weight=1)

        self.make_button(frame, "Séquence 1 : l'escalier", self.lancer_s1, "primary", width=24).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(0, 16)
        )

        s2 = tk.Frame(frame, bg=self.colors["panel"])
        s2.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(0, 16))
        s2.columnconfigure((0, 1), weight=1)

        tk.Label(
            s2,
            text="Séquence 2 : le cercle",
            bg=self.colors["panel"],
            fg=self.colors["text"],
            font=("Arial", 15, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 8))

        tk.Label(s2, text="Rayon (cm)", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=1, column=0, sticky="w", padx=8
        )
        self.rayon_s2 = self.make_entry(s2, "10")
        self.rayon_s2.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 0))

        self.make_button(s2, "Voir simulation cercle", self.simuler_s2, "default").grid(
            row=2, column=1, sticky="ew", padx=8, pady=(4, 0)
        )

        s3 = tk.Frame(frame, bg=self.colors["panel"])
        s3.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(0, 18))
        s3.columnconfigure((0, 1), weight=1)

        tk.Label(
            s3,
            text="Séquence 3 : la rose des vents",
            bg=self.colors["panel"],
            fg=self.colors["text"],
            font=("Arial", 15, "bold"),
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 8))

        tk.Label(s3, text="Longueur flèche (cm)", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=1, column=0, sticky="w", padx=8
        )
        self.longueur_s3 = self.make_entry(s3, "12")
        self.longueur_s3.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 0))

        self.make_button(s3, "Dessiner la flèche Nord", self.dessiner_s3, "success").grid(
            row=2, column=1, sticky="ew", padx=8, pady=(4, 0)
        )

        self.advanced_toggle_btn = self.make_button(
            frame,
            "Afficher réglages avancés",
            self.toggle_advanced_sequences,
            "default",
            width=24,
        )
        self.advanced_toggle_btn.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        self.advanced_sequences_frame = tk.Frame(frame, bg=self.colors["panel"])
        self.advanced_sequences_frame.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(16, 0))
        self.advanced_sequences_frame.columnconfigure((0, 1), weight=1)
        self.advanced_sequences_frame.grid_remove()

        s1 = tk.Frame(self.advanced_sequences_frame, bg=self.colors["panel"])
        s1.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        s1.columnconfigure((0, 1), weight=1)

        tk.Label(s1, text="Correction rotation S1", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=0, sticky="w", padx=8
        )
        self.krot_s1 = self.make_entry(s1, "0.45")
        self.krot_s1.grid(row=1, column=0, sticky="ew", padx=8, pady=(4, 0))
        self.make_button(s1, "Appliquer KROT", self.regler_rotation_s1).grid(
            row=1, column=1, sticky="ew", padx=8, pady=(4, 0)
        )

        gains = tk.Frame(self.advanced_sequences_frame, bg=self.colors["panel"])
        gains.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        gains.columnconfigure((0, 1, 2, 3), weight=1)

        tk.Label(gains, text="KP sync", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=0, sticky="w", padx=8
        )
        self.kp_s1 = self.make_entry(gains, "2.0")
        self.kp_s1.grid(row=1, column=0, sticky="ew", padx=8, pady=(4, 0))

        tk.Label(gains, text="KD sync", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=1, sticky="w", padx=8
        )
        self.kd_s1 = self.make_entry(gains, "0.8")
        self.kd_s1.grid(row=1, column=1, sticky="ew", padx=8, pady=(4, 0))

        tk.Label(gains, text="PWM avance", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=2, sticky="w", padx=8
        )
        self.pwm_avance_s1 = self.make_entry(gains, "95")
        self.pwm_avance_s1.grid(row=1, column=2, sticky="ew", padx=8, pady=(4, 0))

        tk.Label(gains, text="PWM rotation", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=3, sticky="w", padx=8
        )
        self.pwm_rot_s1 = self.make_entry(gains, "110")
        self.pwm_rot_s1.grid(row=1, column=3, sticky="ew", padx=8, pady=(4, 0))

        self.make_button(gains, "Appliquer gains S1", self.regler_gains_s1).grid(
            row=2, column=0, columnspan=4, sticky="ew", padx=8, pady=(10, 0)
        )

        stylo = tk.Frame(self.advanced_sequences_frame, bg=self.colors["panel"])
        stylo.grid(row=2, column=0, columnspan=2, sticky="ew")
        stylo.columnconfigure((0, 1, 2), weight=1)

        tk.Label(stylo, text="K stylo", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=0, sticky="w", padx=8
        )
        self.kstylo_s1 = self.make_entry(stylo, "35")
        self.kstylo_s1.grid(row=1, column=0, sticky="ew", padx=8, pady=(4, 0))

        tk.Label(stylo, text="PWM stylo", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=1, sticky="w", padx=8
        )
        self.pwm_stylo_s1 = self.make_entry(stylo, "65")
        self.pwm_stylo_s1.grid(row=1, column=1, sticky="ew", padx=8, pady=(4, 0))

        self.make_button(stylo, "Appliquer stylo", self.regler_stylo_s1).grid(
            row=1, column=2, sticky="ew", padx=8, pady=(4, 0)
        )

        return frame

    def toggle_advanced_sequences(self):
        self.advanced_sequences_visible = not self.advanced_sequences_visible
        if self.advanced_sequences_visible:
            self.advanced_sequences_frame.grid()
            self.advanced_toggle_btn.config(text="Masquer réglages avancés")
        else:
            self.advanced_sequences_frame.grid_remove()
            self.advanced_toggle_btn.config(text="Afficher réglages avancés")
        self.update_scroll_region()

    def simulation_panel(self, parent):
        frame = self.panel(parent, "Simulation", "Aperçu du trajet prévu du stylo.", "05")
        frame.columnconfigure(0, weight=1)

        self.sim_status = tk.Label(
            frame,
            text=self.sim_title,
            bg=self.colors["panel"],
            fg=self.colors["muted"],
            font=("Arial", 11, "bold"),
            anchor="w",
        )
        self.sim_status.grid(row=2, column=0, sticky="ew", pady=(0, 8))

        self.sim_canvas = tk.Canvas(
            frame,
            height=280,
            bg="#F8FAFC",
            highlightthickness=1,
            highlightbackground=self.colors["border"],
        )
        self.sim_canvas.grid(row=3, column=0, sticky="ew")
        self.sim_canvas.bind("<Configure>", lambda event: self.redraw_simulation())

        controls = tk.Frame(frame, bg=self.colors["panel"])
        controls.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        controls.columnconfigure((0, 1, 2), weight=1)

        self.make_button(controls, "Simuler escalier", self.simuler_s1).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.make_button(controls, "Simuler flèche", self.simuler_s3).grid(row=0, column=1, sticky="ew", padx=8)
        self.make_button(controls, "Effacer", self.clear_simulation, "danger").grid(row=0, column=2, sticky="ew", padx=(8, 0))

        return frame
    def make_entry(self, parent, default_value):
        entry = tk.Entry(
            parent,
            font=("Arial", 15, "bold"),
            justify="center",
            bg=self.colors["input"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"],
            relief="flat",
            bd=0,
        )
        entry.insert(0, default_value)
        return entry

    def telemetry_panel(self, parent):
        frame = self.panel(parent, "Valeurs et caractéristiques", "Chaque bloc met en avant une mesure importante du robot: moteurs, état, réglages et orientation.", "06")
        self.telemetry_container = tk.Frame(frame, bg=self.colors["panel"])
        self.telemetry_container.grid(row=2, column=0, sticky="nsew")
        self.telemetry_container.columnconfigure((0, 1), weight=1)

        self.telemetry_labels = {}
        labels = [
            ("speed_l", "Roue gauche", "Vitesse moteur gauche"),
            ("speed_r", "Roue droite", "Vitesse moteur droit"),
            ("ticks", "Encodeurs", "Ticks mesurés roue gauche / droite"),
            ("distance_robot", "Distance robot", "Distance estimée depuis les encodeurs"),
            ("accel", "Accéléromètre", "Inclinaison et accélération brute"),
            ("gyro", "Gyroscope", "Rotation et stabilité"),
            ("mag", "Magnétomètre", "Champ magnétique mesuré"),
            ("position", "Position stylo", "Estimation X / Y"),
            ("orientation", "Orientation", "Roulis, tangage et direction"),
        ]

        for index, (key, label, description) in enumerate(labels):
            column = index % 2
            row = index // 2
            padx = (0, 10) if column == 0 else (10, 0)

            card = tk.Frame(self.telemetry_container, bg=self.colors["panel_2"], padx=18, pady=16)
            card.grid(row=row, column=column, sticky="nsew", padx=padx, pady=(0 if row == 0 else 18, 0))
            card.columnconfigure(0, weight=1)

            tk.Label(
                card,
                text=label,
                bg=self.colors["panel_2"],
                fg=self.colors["text"],
                font=("Arial", 15, "bold"),
                anchor="w",
            ).grid(row=0, column=0, sticky="w")

            tk.Label(
                card,
                text=description,
                bg=self.colors["panel_2"],
                fg=self.colors["muted"],
                font=("Arial", 10),
                anchor="w",
            ).grid(row=1, column=0, sticky="w", pady=(3, 12))

            value_label = tk.Label(
                card,
                text=self.telemetry_values[key],
                bg=self.colors["panel_2"],
                fg=self.colors["accent"],
                font=("Arial", 18, "bold"),
                anchor="w",
                wraplength=420,
                justify="left",
            )
            value_label.grid(row=2, column=0, sticky="ew")
            self.telemetry_labels[key] = value_label

        frame.rowconfigure(2, weight=1)
        return frame

    def refresh_status(self, text, color):
        self.status.config(text=text, fg=color)
        self.status_dot.itemconfig(self.status_dot_id, fill=color)

    def refresh_telemetry(self):
        for key, value in self.telemetry_values.items():
            self.telemetry_labels[key].config(text=value)

    def lancer_s1(self):
        self.simuler_s1()
        self.envoyer("S1")

    def simuler_s1(self):
        targets = [(0, 20), (-10, 20), (-10, 60)]
        self.start_simulation(
            "Séquence 1 : l'escalier",
            self.simulate_stylus_controller(targets),
            [(0, 0), *targets],
        )

    def simuler_s2(self):
        valeur = self.rayon_s2.get().strip()
        if not self.valeur_numerique_ok(valeur, "rayon"):
            return

        rayon = float(valeur)
        if rayon < 2 or rayon > 20:
            messagebox.showwarning("Rayon invalide", "Le rayon du cercle doit être compris entre 2 cm et 20 cm.")
            return

        points = []
        total = 96
        for i in range(total + 1):
            angle = 2.0 * math.pi * i / total
            points.append((rayon * math.cos(angle), rayon * math.sin(angle)))

        self.start_simulation("Séquence 2 : le cercle", points, points)

    def simuler_s3(self):
        valeur = self.longueur_s3.get().strip()
        if not self.valeur_numerique_ok(valeur, "longueur"):
            return False

        longueur = float(valeur)
        if longueur < 3.5 or longueur > 18:
            messagebox.showwarning("Longueur invalide", "La longueur de la flèche doit être comprise entre 3,5 cm et 18 cm.")
            return False

        tete = min(max(longueur * 0.30, 1.2), 5.0)
        demi_largeur = min(max(longueur * 0.12, 0.6), 2.2)
        epaule_y = longueur - tete
        targets = [
            (0, longueur),
            (-demi_largeur, epaule_y),
            (0, longueur),
            (demi_largeur, epaule_y),
            (0, longueur),
        ]
        self.start_simulation(
            "Séquence 3 : la rose des vents",
            self.simulate_stylus_controller(targets, allow_reverse=True, arrow_mode=True),
            [(0, 0), *targets],
            fixed_bounds=(-4.5, 4.5, 0.0, 18.0),
        )
        return True

    def start_simulation(self, title, points, target_points=None, fixed_bounds=None):
        self.stop_simulation_timer()
        self.sim_title = title
        self.sim_points = points
        self.sim_target_points = target_points or points
        self.sim_fixed_bounds = fixed_bounds
        self.sim_drawn_until = 1 if len(points) > 1 else len(points)
        self.sim_status.config(text=title)
        self.redraw_simulation()
        self.animate_simulation()

    def simulate_stylus_controller(self, targets, allow_reverse=False, arrow_mode=False):
        distance_stylo = 13.0
        entraxe = 14.0
        lookahead = 12.0
        cm_par_pwm_step = 0.00075

        kp_stylo = self.get_float_entry(getattr(self, "kstylo_s1", None), 35.0)
        if arrow_mode:
            kp_stylo = max(kp_stylo, 70.0)
        pwm_stylo = self.get_float_entry(getattr(self, "pwm_stylo_s1", None), 65.0)

        robot_x = 0.0
        robot_y = -distance_stylo
        robot_theta = math.pi / 2.0

        def clamp(value, low, high):
            return max(low, min(high, value))

        def normaliser_rad(angle):
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

        def stylo_pos():
            return (
                robot_x + distance_stylo * math.cos(robot_theta),
                robot_y + distance_stylo * math.sin(robot_theta),
            )

        points = [stylo_pos()]

        for cible_x, cible_y in targets:
            depart_x, depart_y = stylo_pos()
            seg_x = cible_x - depart_x
            seg_y = cible_y - depart_y
            longueur = math.hypot(seg_x, seg_y)
            if longueur < 0.5:
                continue

            ux = seg_x / longueur
            uy = seg_y / longueur
            nx = -uy
            ny = ux
            angle_segment = math.atan2(uy, ux)

            for step in range(1400):
                px, py = stylo_pos()
                rel_x = px - depart_x
                rel_y = py - depart_y
                avance = rel_x * ux + rel_y * uy
                ecart = rel_x * nx + rel_y * ny
                restant = longueur - avance

                if restant < 0.8:
                    break

                angle_desire = angle_segment - math.atan2(ecart, lookahead)
                erreur_cap = normaliser_rad(angle_desire - robot_theta)
                sens = 1
                if allow_reverse and abs(erreur_cap) > math.pi / 2.0:
                    sens = -1
                    erreur_cap = normaliser_rad(angle_desire + math.pi - robot_theta)

                base = max(50.0, pwm_stylo - 15.0) if restant < 5.0 else pwm_stylo
                correction = clamp(kp_stylo * erreur_cap, -35.0, 35.0)

                pwm_g = sens * clamp(base - correction, 45.0, 115.0)
                pwm_d = sens * clamp(base + correction, 45.0, 115.0)

                dist_g = pwm_g * cm_par_pwm_step
                dist_d = pwm_d * cm_par_pwm_step
                dist_c = (dist_g + dist_d) * 0.5
                d_theta = (dist_d - dist_g) / entraxe

                if abs(d_theta) < 1e-5:
                    robot_x += dist_c * math.cos(robot_theta)
                    robot_y += dist_c * math.sin(robot_theta)
                else:
                    rayon = dist_c / d_theta
                    nouveau_theta = robot_theta + d_theta
                    robot_x += rayon * (math.sin(nouveau_theta) - math.sin(robot_theta))
                    robot_y -= rayon * (math.cos(nouveau_theta) - math.cos(robot_theta))
                    robot_theta = nouveau_theta

                if step % 4 == 0:
                    points.append(stylo_pos())

            points.append(stylo_pos())

        return points

    def get_float_entry(self, entry, default):
        if entry is None:
            return default
        try:
            return float(entry.get().strip())
        except ValueError:
            return default

    def clear_simulation(self):
        self.stop_simulation_timer()
        self.sim_title = "Aucune simulation"
        self.sim_points = []
        self.sim_target_points = []
        self.sim_fixed_bounds = None
        self.sim_drawn_until = 0
        self.sim_status.config(text=self.sim_title)
        self.redraw_simulation()

    def stop_simulation_timer(self):
        if self.sim_after_id is not None:
            self.root.after_cancel(self.sim_after_id)
            self.sim_after_id = None

    def animate_simulation(self):
        if self.sim_drawn_until < len(self.sim_points):
            step = max(1, len(self.sim_points) // 90)
            self.sim_drawn_until = min(len(self.sim_points), self.sim_drawn_until + step)
            self.redraw_simulation()
            self.sim_after_id = self.root.after(35, self.animate_simulation)
        else:
            self.sim_after_id = None

    def redraw_simulation(self):
        if not hasattr(self, "sim_canvas"):
            return

        canvas = self.sim_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 400)
        height = max(canvas.winfo_height(), 240)

        canvas.create_rectangle(0, 0, width, height, fill="#F8FAFC", outline="")
        canvas.create_text(16, 16, text="vue du dessus", fill="#64748B", font=("Arial", 10, "bold"), anchor="nw")

        points = self.sim_points
        if not points:
            canvas.create_text(
                width / 2,
                height / 2,
                text="Lance une simulation pour voir le trajet.",
                fill="#475569",
                font=("Arial", 14, "bold"),
            )
            return

        margin = 34
        all_points = points + self.sim_target_points
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        if self.sim_fixed_bounds:
            bound_min_x, bound_max_x, bound_min_y, bound_max_y = self.sim_fixed_bounds
            min_x = min(min(xs), bound_min_x)
            max_x = max(max(xs), bound_max_x)
            min_y = min(min(ys), bound_min_y)
            max_y = max(max(ys), bound_max_y)
        else:
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

        def to_canvas(point):
            x, y = point
            cx = margin + (x - min_x) * scale
            cy = height - margin - (y - min_y) * scale
            return cx, cy

        start = to_canvas(points[0])
        canvas.create_oval(start[0] - 5, start[1] - 5, start[0] + 5, start[1] + 5, fill="#22C55E", outline="")
        canvas.create_text(start[0] + 9, start[1] - 10, text="départ", fill="#15803D", font=("Arial", 9, "bold"), anchor="w")

        flat_target = []
        for point in self.sim_target_points:
            flat_target.extend(to_canvas(point))
        if len(flat_target) >= 4:
            canvas.create_line(*flat_target, fill="#94A3B8", width=3, dash=(6, 5), smooth=False)
            canvas.create_text(
                width - 16,
                16,
                text="gris = objectif / bleu = simulation robot",
                fill="#64748B",
                font=("Arial", 10, "bold"),
                anchor="ne",
            )

        visible = points[: max(1, self.sim_drawn_until)]
        flat_visible = []
        for point in visible:
            flat_visible.extend(to_canvas(point))
        if len(flat_visible) >= 4:
            canvas.create_line(*flat_visible, fill="#0EA5E9", width=5, capstyle=tk.ROUND, joinstyle=tk.ROUND, smooth=False)

        robot_point = visible[-1]
        rx, ry = to_canvas(robot_point)
        canvas.create_oval(rx - 8, ry - 8, rx + 8, ry + 8, fill="#EF4444", outline="")
        canvas.create_text(rx + 12, ry, text="stylo", fill="#991B1B", font=("Arial", 9, "bold"), anchor="w")

    def connecter(self):
        if self.is_connecting:
            return

        if self.client is not None and self.client.is_connected:
            self.refresh_status("Déjà connecté", self.colors["green"])
            return

        self.is_connecting = True
        self.connect_btn.config(state="disabled", text="Recherche...")
        self.refresh_status("Recherche...", self.colors["orange"])
        asyncio.run_coroutine_threadsafe(self._connecter(), self.loop)

    async def _connecter(self):
        try:
            print("Recherche du robot...")
            devices = await BleakScanner.discover(timeout=8)

            robot = None
            for device in devices:
                print("Bluetooth trouvé :", device.name, device.address)
                if device.name and NOM_ROBOT.upper() in device.name.upper():
                    robot = device
                    break

            if robot is None:
                self.root.after(0, self.on_connection_failed, "Robot DRAWBOT introuvable.\nVérifie qu'il est allumé et disponible.")
                return

            self.client = BleakClient(robot.address)
            await self.client.connect()
            await self.client.start_notify(CHAR_UUID_TX, self.notification_handler)

            self.root.after(0, self.on_connected, robot.address)
            print("Connecté à DRAWBOT")

        except Exception as e:
            print("Erreur connexion :", e)
            self.root.after(0, self.on_connection_failed, str(e))

    def on_connected(self, address):
        self.is_connecting = False
        self.connect_btn.config(state="normal", text="Connecter")
        self.refresh_status("Connecté", self.colors["green"])

    def on_connection_failed(self, message):
        self.is_connecting = False
        self.connect_btn.config(state="normal", text="Connecter")
        self.refresh_status("Connexion impossible", self.colors["red"])
        messagebox.showerror("Erreur connexion", message)

    def deconnecter(self):
        asyncio.run_coroutine_threadsafe(self._deconnecter(), self.loop)

    async def _deconnecter(self):
        try:
            if self.client is not None and self.client.is_connected:
                await self.client.disconnect()
            self.root.after(0, self.on_disconnected)
        except Exception as e:
            print("Erreur déconnexion :", e)
            self.root.after(0, lambda: messagebox.showerror("Erreur déconnexion", str(e)))

    def on_disconnected(self):
        self.refresh_status("Déconnecté", self.colors["red"])

    def notification_handler(self, sender, data):
        try:
            msg = data.decode("utf-8")

            if msg.startswith("S3DATA:"):
                valeurs = msg[7:].split(",")
                cap = valeurs[0] if len(valeurs) > 0 else "---"
                ticks_gauche = valeurs[1] if len(valeurs) > 1 else "---"
                ticks_droite = valeurs[2] if len(valeurs) > 2 else "---"
                etat = valeurs[3] if len(valeurs) > 3 else "---"
                etape = valeurs[4] if len(valeurs) > 4 else "---"
                point = valeurs[5] if len(valeurs) > 5 else "---"
                cap_robot = valeurs[6] if len(valeurs) > 6 else cap
                erreur_nord = valeurs[7] if len(valeurs) > 7 else "---"
                phase_nord = valeurs[8] if len(valeurs) > 8 else "---"
                sens_cap = valeurs[9] if len(valeurs) > 9 else "---"

                self.telemetry_values.update({
                    "mag": f"cap brut={cap} deg / cap robot={cap_robot} deg",
                    "ticks": f"G={ticks_gauche} / D={ticks_droite}",
                    "distance_robot": f"S3 étape={etape} / point={point}",
                    "orientation": f"État={etat} / cap={cap_robot} deg / erreur N={erreur_nord} deg / phase={phase_nord} / sens={sens_cap}",
                })
                self.root.after(0, self.refresh_telemetry)
                return

            if msg.startswith("S1DATA:"):
                valeurs = msg[7:].split(",")
                ticks_gauche = valeurs[0] if len(valeurs) > 0 else "---"
                ticks_droite = valeurs[1] if len(valeurs) > 1 else "---"
                etat = valeurs[2] if len(valeurs) > 2 else "---"
                actif = valeurs[3] if len(valeurs) > 3 else "---"
                krot = valeurs[4] if len(valeurs) > 4 else "---"
                kp = valeurs[5] if len(valeurs) > 5 else "---"
                kd = valeurs[6] if len(valeurs) > 6 else "---"
                pwm_base = valeurs[7] if len(valeurs) > 7 else "---"
                pwm_rot = valeurs[8] if len(valeurs) > 8 else "---"
                kstylo = valeurs[9] if len(valeurs) > 9 else "---"
                pwm_stylo = valeurs[10] if len(valeurs) > 10 else "---"

                self.telemetry_values.update({
                    "ticks": f"G={ticks_gauche} / D={ticks_droite}",
                    "distance_robot": f"S1 actif={actif}",
                    "orientation": f"État escalier={etat} / KROT={krot} / KP={kp} / KD={kd} / PWM={pwm_base}/{pwm_rot} / stylo={kstylo}/{pwm_stylo}",
                })
                self.root.after(0, self.refresh_telemetry)
                return

            if not msg.startswith("DATA:"):
                return

            valeurs = msg[5:].split(",")

            if len(valeurs) < 11:
                return

            ax = float(valeurs[0])
            ay = float(valeurs[1])
            az = float(valeurs[2])

            gx = float(valeurs[3])
            gy = float(valeurs[4])
            gz = float(valeurs[5])

            mx = valeurs[6]
            my = valeurs[7]
            mz = valeurs[8]

            v_gauche = valeurs[9]
            v_droite = valeurs[10]
            ticks_gauche = valeurs[11] if len(valeurs) > 11 else "---"
            ticks_droite = valeurs[12] if len(valeurs) > 12 else "---"
            distance_robot = valeurs[13] if len(valeurs) > 13 else "---"

            now = time.time()

            if self.last_imu_time is None:
                self.last_imu_time = now
                return

            dt = now - self.last_imu_time
            self.last_imu_time = now

            if dt <= 0 or dt > 1:
                return

            if self.calibration_count < 50:
                self.ax_offset += ax
                self.ay_offset += ay
                self.gz_offset += gz
                self.calibration_count += 1
                return

            if self.calibration_count == 50:
                self.ax_offset /= 50
                self.ay_offset /= 50
                self.gz_offset /= 50
                self.calibration_count += 1
                return

            ax_corr = ax - self.ax_offset
            ay_corr = ay - self.ay_offset
            gz_corr = gz - self.gz_offset

            roulis = math.degrees(math.atan2(ay, az))
            tangage = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

            gz_deg_s = gz_corr * 0.00875
            self.yaw += gz_deg_s * dt
            self.yaw = (self.yaw + 180) % 360 - 180

            ax_ms2 = ax_corr * 0.000061 * 9.81
            ay_ms2 = ay_corr * 0.000061 * 9.81

            if abs(ax_ms2) < 0.08:
                ax_ms2 = 0
            if abs(ay_ms2) < 0.08:
                ay_ms2 = 0

            self.vx += ax_ms2 * dt
            self.vy += ay_ms2 * dt

            self.vx *= 0.90
            self.vy *= 0.90

            self.pos_x += self.vx * dt * 100
            self.pos_y += self.vy * dt * 100

            self.pos_x = max(min(self.pos_x, 200), -200)
            self.pos_y = max(min(self.pos_y, 200), -200)

            self.telemetry_values = {
                "accel": f"{int(ax)} / {int(ay)} / {int(az)}",
                "gyro": f"{int(gx)} / {int(gy)} / {int(gz)}",
                "mag": f"{mx} / {my} / {mz}",
                "speed_l": f"{v_gauche} ticks/s",
                "speed_r": f"{v_droite} ticks/s",
                "ticks": f"G={ticks_gauche} / D={ticks_droite}",
                "distance_robot": f"{distance_robot} cm",
                "position": f"X={self.pos_x:.1f} cm / Y={self.pos_y:.1f} cm",
                "orientation": f"G/D={roulis:.1f} deg / A/R={tangage:.1f} deg / Direction={self.yaw:.1f} deg",
            }

            self.root.after(0, self.refresh_telemetry)

        except Exception as e:
            print("Erreur réception capteurs :", e)

    def envoyer(self, commande):
        asyncio.run_coroutine_threadsafe(self._envoyer(commande), self.loop)

    async def _envoyer(self, commande):
        try:
            if self.client is None or not self.client.is_connected:
                self.root.after(0, lambda: messagebox.showwarning("Erreur", "Robot non connecté"))
                return

            await self.client.write_gatt_char(CHAR_UUID_RX, commande.encode("utf-8"), response=True)
            print("Envoyé :", commande)

        except Exception as e:
            print("Erreur envoi :", e)
            self.root.after(0, lambda: messagebox.showerror("Erreur envoi", str(e)))

    def avancer_cm(self):
        valeur = self.distance.get().strip()
        if self.valeur_numerique_ok(valeur, "distance"):
            self.envoyer("A:" + valeur)

    def reculer_cm(self):
        valeur = self.distance.get().strip()
        if self.valeur_numerique_ok(valeur, "distance"):
            self.envoyer("R:" + valeur)

    def tourner_angle(self):
        valeur = self.angle.get().strip()
        if self.valeur_numerique_ok(valeur, "angle"):
            self.envoyer("T:" + valeur)

    def regler_rotation_s1(self):
        valeur = self.krot_s1.get().strip()
        if not self.valeur_numerique_ok(valeur, "correction rotation"):
            return

        krot = float(valeur)
        if krot < 0.10 or krot > 1.20:
            messagebox.showwarning("Correction invalide", "La correction rotation doit être entre 0.10 et 1.20.")
            return

        self.envoyer("KROT:" + valeur)

    def regler_gains_s1(self):
        kp = self.kp_s1.get().strip()
        kd = self.kd_s1.get().strip()
        pwm_avance = self.pwm_avance_s1.get().strip()
        pwm_rot = self.pwm_rot_s1.get().strip()

        for valeur, nom in [
            (kp, "KP sync"),
            (kd, "KD sync"),
            (pwm_avance, "PWM avance"),
            (pwm_rot, "PWM rotation"),
        ]:
            if not self.valeur_numerique_ok(valeur, nom):
                return

        self.envoyer("KP:" + kp)
        self.envoyer("KD:" + kd)
        self.envoyer("PWM:" + pwm_avance + "," + pwm_rot)

    def regler_stylo_s1(self):
        kstylo = self.kstylo_s1.get().strip()
        pwm_stylo = self.pwm_stylo_s1.get().strip()

        if not self.valeur_numerique_ok(kstylo, "K stylo"):
            return
        if not self.valeur_numerique_ok(pwm_stylo, "PWM stylo"):
            return

        self.envoyer("KSTYLO:" + kstylo)
        self.envoyer("PSTYLO:" + pwm_stylo)

    def trouver_nord(self):
        self.envoyer("FINDN")

    def dessiner_s3(self):
        if self.simuler_s3():
            self.envoyer("DRAW3:" + self.longueur_s3.get().strip())

    def lancer_s3(self):
        valeur = self.longueur_s3.get().strip()
        if self.valeur_numerique_ok(valeur, "longueur"):
            self.envoyer("S3:" + valeur)

    def valeur_numerique_ok(self, valeur, nom):
        try:
            float(valeur)
            return True
        except ValueError:
            messagebox.showwarning("Valeur invalide", f"La valeur de {nom} doit être un nombre.")
            return False

    def quitter(self):
        async def disconnect_and_stop():
            try:
                if self.client is not None and self.client.is_connected:
                    await self.client.disconnect()
            finally:
                self.loop.call_soon_threadsafe(self.loop.stop)
                self.root.after(0, self.root.destroy)

        asyncio.run_coroutine_threadsafe(disconnect_and_stop(), self.loop)


if __name__ == "__main__":
    root = tk.Tk()
    app = DrawbotGUI(root)
    root.mainloop()
