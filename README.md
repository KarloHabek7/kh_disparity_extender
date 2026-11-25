# kh_disparity_extender (ROS 2 Jazzy)

**Kolegij:** Projektiranje autonomnih sustava (PAS) — Fakultet strojarstva i brodogradnje (FSB)  
**Autor:** Karlo Habek  
**Referenca algoritma:** [Nathan Otterness: The Disparity Extender Algorithm (UNC, F1TENTH 2019)](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html)

---

## 1. Opis implementacije

Ovaj paket sadrži samostalnu ROS 2 implementaciju **Disparity Extender** algoritma za reaktivno izbjegavanje prepreka i autonomno vođenje F1TENTH trkaćeg vozila.

Osnovna ideja algoritma je detekcija diskontinuiteta ("disparity" ili skokova u udaljenosti) u 2D LiDAR očitanjima. Svaki nagli skok udaljenosti između dvije susjedne zrake signalizira rub prepreke (ugao zida ili prepreku na stazi). Zbog konačne širine vozila ($w_{\text{car}}$), vožnja tik uz rub prepreke dovela bi do sudara bočne strane vozila. Zbog toga se prepreka "proširuje" (eng. *extend*) na daljoj strani za polovicu širine vozila uvećanu za sigurnosnu marginu. Nakon proširenja svih rubova, vozilo bira najdalji slobodan smjer kretanja i prilagođava brzinu.

---

## 2. Matematički koraci algoritma

Arhitektura koda je podijeljena u dva modula:
- `kh_disparity_extender/algorithm.py`: Čiste NumPy funkcije (matematička jezgra, neovisna o ROS-u).
- `kh_disparity_extender/disparity_extender_node.py`: ROS 2 čvor koji prima `/scan`, prosljeđuje ga algoritmu, objavljuje `/drive` i šalje dijagnostičke podatke u RViz.

### Korak 1: Preprocesiranje LiDAR podataka (`preprocess_scan`)
1. Ograničavanje vidnog polja: uzimaju se samo prednje zrake unutar simetričnog kuta ispred vozila (npr. $\pm 90^\circ$ za $180^\circ$ FOV), jer zrake iza vozila ne utječu na prednje planiranje.
2. Zamjena `NaN` i `Inf` očitanja s gornjom granicom dometa (`max_lidar_range`).
3. Ograničavanje svih mjerenja na `max_lidar_range`.
4. Zaglađivanje 1D medijan filtrom radi uklanjanja šuma pojedinačnih zraka.

### Korak 2: Detekcija i proširenje disparityja (`detect_and_extend_disparities`)
Prolaskom kroz niz zraka detektiraju se skokovi:
$$|r_{i+1} - r_i| > \delta_{\text{threshold}}$$

Za svaki skok:
- Određuje se bliža udaljenost (prepreka): $r_{\text{close}} = \min(r_i, r_{i+1})$.
- Potrebna geometrijska širina maskiranja:
  $$d_{\text{clearance}} = \frac{w_{\text{car}}}{2} + d_{\text{margin}}$$
- Broj zraka koje je potrebno prebrisati na daljoj strani:
  $$N_{\text{beams}} = \left\lceil \frac{d_{\text{clearance}}}{r_{\text{close}} \cdot \Delta\theta} \right\rceil$$
- Ako je $r_{i+1} > r_i$ (prepreka je na $i$), prebrisuju se zrake prema naprijed: od $i+1$ do $i+1+N_{\text{beams}}$ s vrijednošću $r_{\text{close}}$.
- Ako je $r_i > r_{i+1}$ (prepreka je na $i+1$), prebrisuju se zrake prema natrag: od $i-N_{\text{beams}}+1$ do $i+1$ s vrijednošću $r_{\text{close}}$.

### Korak 3: Odabir najboljeg smjera kretanja (`find_furthest_direction`)
- Pronalazi se maksimalna dubina na modificiranom scanu ($r_{\max} = \max(r)$).
- Pronalaze se sve zrake koje dosežu prag dubine:
  $$r \ge \alpha \cdot r_{\max}$$
  gdje je $\alpha = 0.85$ (parametar `far_ratio`, odnosno 85% maksimalnog dometa).
- Te "duboke" zrake se grupiraju u kontinuirane prolaze; bira se najširi prolaz, a ciljni kut upravljanja postavlja se u **točno središte najšireg prolaza**.

### Korak 4: Dinamička brzina (`calculate_speed`)
Brzina se dinamički skalira:
- Na ravnim dijelovima (mali kut skretanja i čist prolaz ispred) vozilo vozi maksimalnom brzinom (`max_speed = 4.0 m/s`).
- U oštrim zavojima brzina glatko pada prema minimalnoj (`min_speed = 1.5 m/s`).
- Ako je prepreka direktno ispred vozila na udaljenosti manjoj od `slow_down_distance`, brzina se dodatno reducira radi sprječavanja naleta.

---

## 3. Popis dodanih poboljšanja (Improvements)

U odnosu na naivnu implementaciju dodana su sljedeća poboljšanja:

1. **1D Medijan filtar šuma:** Uklanja pojedinačne lažne zrake ("salt & pepper" šum) prije detekcije skokova, čime se eliminira pojava lažnih disparityja.
2. **Središte najdubljeg kontinuiranog prozora (Center of Deepest Window):** Klasični `argmax` često bira rubnu zraku ako je više zraka jednako udaljeno (npr. sve na maksimalnom dometu), što uzrokuje oscilacije kotača. Naše rješenje pronalazi najširi pojas dubokih zraka i cilja u njegovu geometrijsku sredinu, čime vozilo vozi točno po sredini staze.
3. **Dvostruko profiliranje brzine:** Brzina ovisi i o kutu upravljanja i o frontalnoj udaljenosti (`forward_distance`), što omogućuje visoke brzine na ravnicama uz pouzdano i stabilno kočenje pred zavojima.
4. **RViz vizualizacija u stvarnom vremenu:** Čvor objavljuje `/scan_proc` (preprocesirani podaci), `/scan_extended` (proširene prepreke gdje se jasno vidi stvorena "sjena") i `/target_marker` (zelena 3D strelica smjera kretanja).
5. **Čisto zaustavljanje pri prekidu (Graceful Stop):** Prilikom pritiska na `Ctrl+C`, čvor presreće signal i prije gašenja šalje komandu s brzinom 0.0 m/s, sprječavajući da se vozilo nekontrolirano nastavi kretati.

---

## 4. Parametrizacija preko YAML datoteke (`config/params.yaml`)

Svi ključni parametri mogu se mijenjati bez rekompajliranja:

| Parametar | Zadana vrijednost | Opis |
|---|---|---|
| `car_width` | `0.31` m | Stvarna širina vozila F1TENTH |
| `safety_margin` | `0.15` m | Dodatna sigurnosna margina uz prepreku |
| `max_steering_angle` | `0.34` rad | Maksimalni mehanički kut upravljanja (~19.5°) |
| `disparity_threshold` | `0.35` m | Minimalni skok udaljenosti koji se smatra rubom prepreke |
| `fov_degrees` | `180.0`° | Kutno vidno polje ispred vozila (-90° do +90°) |
| `max_lidar_range` | `15.0` m | Gornja granica dometa LiDAR zraka |
| `filter_window_size` | `3` | Veličina prozora medijan filtra (1 = isključeno) |
| `far_ratio` | `0.85` | Udio maksimalne udaljenosti za duboki prozor (85%) |
| `max_speed` | `4.0` m/s | Najveća brzina na ravnim segmentima |
| `min_speed` | `1.5` m/s | Minimalna brzina u oštrim zavojima |
| `speed_steer_threshold` | `0.10` rad | Kut skretanja iznad kojeg počinje kočenje |
| `slow_down_distance` | `3.0` m | Udaljenost ispred vozila ispod koje se usporava |
| `scan_topic` | `"scan"` | Naziv dolazne LiDAR teme |
| `drive_topic` | `"drive"` | Naziv odlazne teme upravljanja |

---

## 5. Upute za build i pokretanje

### 1. Pokretanje simulator kontejnera
U mapi `PAS-roboracer/vjezbe-primjeri`:
```bash
docker compose up -d
```

### 2. Build paketa unutar kontejnera
Uđite u `roboracer-sim` kontejner:
```bash
docker exec -it roboracer-sim bash
```
Unutar kontejnera pokrenite build:
```bash
cd /sim_ws
colcon build --symlink-install --packages-select kh_disparity_extender
source install/setup.bash
```

### 3. Pokretanje simulatora
U prvom terminalu unutar kontejnera pokrenite simulator na željenoj mapi:

**Opcija A: F1 Trkaća staza Spielberg (preporučeno za brzinu i duge zavoje)**
U datoteci `vjezbe-primjeri/config/sim_config.yaml` postavite:
```yaml
map_path: 'maps/Spielberg'
sx: -0.04
sy: -0.85
stheta: 3.40
```
I pokrenite:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=/sim_ws/config/sim_config.yaml
```

**Opcija B: Hodnik s preprekama (`levine_obs`)**
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=/sim_ws/config/sim_config.yaml map_path:=maps/levine_obs
```

### 4. Pokretanje Disparity Extender čvora
U drugom terminalu unutar kontejnera:
```bash
docker exec -it roboracer-sim bash
source /sim_ws/install/setup.bash
ros2 launch kh_disparity_extender disparity_extender.launch.py
```

*Napomena:* Ako želite pokrenuti čvor s alternativnom konfiguracijom, možete proslijediti putanju:
```bash
ros2 launch kh_disparity_extender disparity_extender.launch.py params_file:=/putanja/do/custom_params.yaml
```

### 5. Prikaz u RViz-u (noVNC preglednik)
1. U web pregledniku otvorite: `http://localhost:8080/vnc.html` i kliknite **Connect**.
2. U RViz sučelju kliknite **Add -> By topic** i dodajte:
   - `/scan_proc` (LaserScan) — preprocesirane zrake ispred auta
   - `/scan_extended` (LaserScan) — zrake nakon proširenja disparityja (vidljive sjene prepreka)
   - `/target_marker` (Marker) — zelena strelica ciljnog smjera upravljanja

---

## 6. Pokretanje jediničnih testova (Unit Tests)

Paket dolazi s testovima za verifikaciju matematičkih funkcija u `test/test_algorithm.py`:
```bash
PYTHONPATH=/sim_ws/src/student/kh_disparity_extender python3 -m pytest /sim_ws/src/student/kh_disparity_extender/test/test_algorithm.py
```
Svi testovi prolaze i potvrđuju ispravnost preprocesiranja, proširenja prepreka u oba smjera, odabira cilja i profiliranja brzine.
