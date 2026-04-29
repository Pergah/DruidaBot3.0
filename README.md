# DruidaBot 3.0

Firmware y frontend para el controlador de cultivo DruidaBot 3.0 (ESP32-S3 Waveshare ETH-8DI-8RO).

## Archivos de release

| Archivo | Descripción |
|---|---|
| `backend.ino.bin` | Firmware compilado — se sube vía OTA al ESP32 |
| `frontend.bin` | Filesystem FFat con la webapp — se sube vía OTA al ESP32 |

---

## Cómo modificar la webapp y generar `frontend.bin`

### Requisitos

- Python 3.x instalado y en el PATH
- `mkfatfs.exe` en `C:\Tools\mkfatfs\mkfatfs.exe`
  - Descarga: https://github.com/marcmerlin/esp32_fatfsimage/releases
- Git con acceso al repo

### Estructura de la webapp

```
data/
  index.html   <- estructura HTML principal
  style.css    <- estilos
  app.js       <- logica del frontend (fetch al ESP32)
```

### Flujo de trabajo

1. Editar los archivos en `data/`
2. Doble clic en `push_frontend.bat`

O desde terminal:

```
python make_frontend.py
```

El script hace todo:
1. Crea la imagen FAT con mkfatfs (tamanio correcto con overhead WL)
2. Aplica la capa Wear-Leveling de ESP-IDF 5.x
3. Clona/actualiza el repo de GitHub
4. Commitea y pushea `frontend.bin`

### Solo generar el .bin sin pushear

```
python make_frontend.py --only-build
```

Guarda `frontend.bin` en la carpeta del proyecto sin tocar Git.

### Por que NO usar mkfatfs directamente

El ESP32 usa FFat con una capa Wear-Leveling sobre el filesystem.
Si generás el `.bin` solo con mkfatfs directo, el ESP32 no puede montar el
filesystem y tira el error `index.html no cargado en RAM`.

El proceso correcto (que hace el script):

```
mkfatfs -s 10321920   ->  imagen FAT cruda  (NO el tamanio total de la particion)
                       ->  wrap wl_config_t + wl_state_t  (ESP-IDF 5.x)
                       ->  imagen final: 10354688 bytes
```

### Subir el firmware (backend)

Compilar desde Arduino IDE. El `.ino.bin` de la carpeta `build/` se renombra a
`backend.ino.bin` y se sube al repo.

---

## OTA URLs (en `config.h`)

```c
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/backend.ino.bin"
#define OTA_FFAT_URL     "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/frontend.bin"
```
