## 📋 Projekt Célja

A projekt célja, hogy a Turtlesim szimulátorban egy teknős segítségével az **"ROS"** szöveget rajzoljuk ki. A kód három betűt ("R", "O", "S") implementál, amelyeket a teknős vonalak és fordulások kombinációjával rajzol meg.

## 🔧 Előfeltételek

- **Operációs rendszer:** Ubuntu 22.04 (vagy WSL Windows 11 alatt)
- **ROS 2:** Humble verzió
- **Turtlesim:** Telepítve a ROS 2-vel
- **Visual Studio Code:** ROS 2 és Python

## 🚀 Telepítés

### 1. ROS 2 Telepítése

#### Lépések:

1. **Nyiss egy terminált** (`Ctrl + Alt + T`)

2. **Csomaglisták frissítése** és állítsd be a lokalizációt:
   ```bash
   sudo apt update
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

3. **Szükséges eszközök telepítése**:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **ROS 2 Humble Desktop telepítése** és a fejlesztői eszközöket:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   ```

5. **Automatikus betöltés:** Add hozzá a `.bashrc` fájlhoz:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### 2. Turtlesim Telepítése

A Turtlesim szimulátor:

```bash
sudo apt install ros-humble-turtlesim
```

### 3. Visual Studio Code Beállítása

#### Szükséges Kiegészítők:
- **ROS** (`ms-iot.vscode-ros`): ROS 2 támogatás
- **Python** (`ms-python.python`): Python kód támogatása


## ⚙️ A Kód Működése

A projekt egy ROS 2 node-ot (`turtle_controller`) tartalmaz, amely a Turtlesim szimulátorban irányítja a teknősöt az **"ROS"** szöveg megrajzolására. A kód Pythonban íródott.

### Főbb Metódusok:

| Metódus       | Leírás |
|---------------|--------|
| `__init__`     | Inicializálja a ROS csomópontot, létrehozza a teleportáláshoz és tollbeállításhoz szükséges klienseket |
| `call_service` | Segédmetódus, amely ROS szolgáltatásokat hív meg és megvárja az eredményt |
| `set_pen`      | Beállítja a teknős tollának színét (RGB), vastagságát és állapotát (ki/be) |
| `teleport`     | Abszolút pozícióba (x, y) teleportálja a teknőst, opcionálisan megadott iránnyal (theta fokban) |
| `draw_line`    | Egyenes vonalat rajzol az (x1, y1) pozícióból az (x2, y2) pozícióba, megadott színnel és vastagsággal |
| `draw_R`       | Megrajzolja az "R" betűt az (sx, sy) kezdőpozícióból, piros színben |
| `draw_O`       | Megrajzolja az "O" betűt az (sx, sy) kezdőpozícióból, zöld színben |
| `draw_S`       | Megrajzolja az "S" betűt az (sx, sy) kezdőpozícióból, kék színben |

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

### 4. (vagy)Futtasd a launch fájlt.
```bash
ros2 launch turtlesim_fractal turtlesim_text_launch.py
```


## 🎨 Eredmény

A teknős megrajzolja az **"ROS"** szöveget a szimulátorban:
- **"R"** - piros színnel
- **"O"** - zöld színnel  
- **"S"** - kék színnel