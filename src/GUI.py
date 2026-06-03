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

        self.build_interface()
        self.refresh_status("Déconnecté", self.colors["red"])
        self.refresh_telemetry()

    def build_interface(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = tk.Frame(self.root, bg=self.colors["bg"], padx=28, pady=22)
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

        status_box = tk.Frame(header, bg=self.colors["panel"], padx=14, pady=10)
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

        scroll_area = tk.Frame(self.root, bg=self.colors["bg"])
        scroll_area.grid(row=1, column=0, sticky="nsew")
        scroll_area.columnconfigure(0, weight=1)
        scroll_area.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(scroll_area, bg=self.colors["bg"], highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        scrollbar = tk.Scrollbar(scroll_area, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        content = tk.Frame(self.canvas, bg=self.colors["bg"], padx=34, pady=10)
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
        self.circle_panel(content).grid(row=4, column=0, sticky="ew", pady=(0, 26))
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
        frame = tk.Frame(parent, bg=self.colors["panel"], padx=26, pady=24)
        frame.columnconfigure(0, weight=1)

        title_row = tk.Frame(frame, bg=self.colors["panel"])
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
        manual = tk.Frame(frame, bg=self.colors["panel"])
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
        frame = self.panel(parent, "Séquence 1 - Escalier", "Lance le parcours programmé dans le firmware: avancer, tourner, avancer, tourner, puis avancer.", "04")
        frame.columnconfigure(0, weight=1)

        self.make_button(frame, "Séquence 1 - Escalier", lambda: self.envoyer("S1"), "primary", width=24).grid(
            row=2, column=0, sticky="ew"
        )

        return frame

    def circle_panel(self, parent):
        frame = self.panel(parent, "Séquence 2 - Cercle paramétrable", "Dessiner un cercle dont le rayon du tracé est envoyé depuis l'ordinateur.", "05")
        frame.columnconfigure((0, 1), weight=1)

        form = tk.Frame(frame, bg=self.colors["panel"])
        form.grid(row=2, column=0, columnspan=2, sticky="ew")
        form.columnconfigure((0, 1), weight=1)

        tk.Label(form, text="Rayon du tracé au stylo (cm)", bg=self.colors["panel"], fg=self.colors["muted"], font=("Arial", 10)).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=8
        )
        self.rayon_cercle = self.make_entry(form, "10")
        self.rayon_cercle.grid(row=1, column=0, sticky="ew", padx=8, pady=(4, 12))

        self.make_button(form, "Dessiner cercle", self.dessiner_cercle, "primary").grid(
            row=1, column=1, sticky="ew", padx=8, pady=(4, 12)
        )

        tk.Label(
            frame,
            text="Rayons < 13 cm: mode expérimental par petits segments devant le robot.",
            bg=self.colors["panel"],
            fg=self.colors["muted"],
            font=("Arial", 10),
            justify="left",
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))

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
        frame = self.panel(parent, "Valeurs et caractéristiques", "Chaque bloc met en avant une mesure importante du robot: moteurs, centrale inertielle, magnétomètre, position et orientation.", "06")
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

    def dessiner_cercle(self):
        valeur = self.rayon_cercle.get().strip()
        if not self.valeur_numerique_ok(valeur, "rayon"):
            return

        rayon = float(valeur)
        if rayon < 2 or rayon > 20:
            messagebox.showwarning("Rayon invalide", "Le rayon du cercle doit être compris entre 2 cm et 20 cm.")
            return

        self.envoyer("C:" + valeur)

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
