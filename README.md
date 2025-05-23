## 📋 Projekt Célja

A projekt célja, hogy a Turtlesim szimulátorban egy teknős segítségével az **"ROS"** szöveget rajzoljuk ki. A kód három betűt ("R", "O", "S") implementál, amelyeket a teknős vonalak és fordulások kombinációjával rajzol meg. A projekt könnyen bővíthető további betűk vagy mintázatok hozzáadásával.

## 🔧 Előfeltételek

- **Operációs rendszer:** Ubuntu 22.04 (vagy WSL Windows 11 alatt)
- **ROS 2:** Humble verzió
- **Turtlesim:** Telepítve a ROS 2-vel
- **Visual Studio Code:** ROS 2 és Python kiegészítőkkel
- **Git:** Verziókövetéshez (opcionális)

## 🚀 Telepítés

### 1. ROS 2 Telepítése

Telepítsd a ROS 2 Humble verzióját az Ubuntu rendszeredre.

#### Lépések:

1. **Nyiss egy terminált** (`Ctrl + Alt + T`)

2. **Frissítsd a csomaglistát** és állítsd be a lokalizációt:
   ```bash
   sudo apt update
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

3. **Telepítsd a szükséges eszközöket** és állítsd be a ROS 2 tárolót:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **Telepítsd a ROS 2 Humble Desktop verzióját** és a fejlesztői eszközöket:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   ```

5. **Ellenőrizd a telepítést:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_py talker
   ```
   Ha üzeneteket látsz a terminálban, a telepítés sikeres.

6. **Automatikus betöltés:** Add hozzá a `.bashrc` fájlhoz:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### 2. Turtlesim Telepítése

A Turtlesim szimulátorral rajzoljuk a szöveget:

```bash
sudo apt install ros-humble-turtlesim
```

### 3. Visual Studio Code Beállítása

#### Telepítés:
```bash
sudo snap install code --classic
```

#### Szükséges Kiegészítők:
- **ROS** (`ms-iot.vscode-ros`): ROS 2 támogatás
- **Python** (`ms-python.python`): Python kód támogatása
- **GitLens** (opcionális): Verziókövetéshez

#### ROS 2 integráció:
Nyisd meg a projekt mappáját: `File > Open Folder...` > válaszd a `~/ros2_ws`-t

## 📁 Projekt Beállítása

### 1. ROS 2 Workspace Létrehozása

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. ROS 2 Csomag Létrehozása

```bash
ros2 pkg create --build-type ament_python --node-name turtle_controller turtlesim_text
```

## ⚙️ A Kód Működése

A projekt egy ROS 2 node-ot (`turtle_controller`) tartalmaz, amely a Turtlesim szimulátorban irányítja a teknősöt az **"ROS"** szöveg megrajzolására. A kód Pythonban íródott.

### Főbb Metódusok:

| Metódus | Leírás |
|---------|--------|
| `__init__` | Inicializálja a node-ot, létrehozza a publishert és klienseket |
| `set_pen` | Beállítja a teknős tollának színét (RGB), vastagságát és állapotát |
| `set_position` | Abszolút pozícióba teleportálja a teknősöt |
| `move_forward` | Előre mozgatja a teknősöt megadott távolságra |
| `turn` | Elforgatja a teknősöt adott szögben |
| `set_heading` | Beállítja a teknős irányát |
| `draw_R` | Megrajzolja az "R" betűt |
| `draw_O` | Megrajzolja az "O" betűt |
| `draw_S` | Megrajzolja az "S" betűt |

A `main` függvényben az "R", "O" és "S" betűk rajzolása sorrendben történik, különböző kezdőpozíciókból.

## 🎯 Projekt Futtatása

### 1. Buildeld a Workspace-t

A VS Code termináljában:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 2. Forrásold a Környezetet
```bash
source install/setup.bash
```

### 3. Indítsd el a Turtlesim Szimulátort

Egy **külön terminálban**:
```bash
ros2 run turtlesim turtlesim_node
```

### 4. Futtasd a Controllert

A VS Code termináljában:
```bash
ros2 run turtlesim_text turtle_controller
```

## 🎨 Eredmény

A teknős megrajzolja az **"ROS"** szöveget a szimulátorban:
- **"R"** - piros színnel
- **"O"** - zöld színnel  
- **"S"** - kék színnel

## 🔄 Bővítési Lehetőségek

- További betűk implementálása
- Színek testreszabása
- Betűméret módosítása
- Komplex mintázatok rajzolása

---

## 📝 Megjegyzések

Ez a projekt remek kiindulópont a ROS 2 és Turtlesim megismeréséhez, valamint a robotika alapjainak elsajátításához.