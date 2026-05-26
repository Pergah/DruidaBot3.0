# Formato de Assets — Druida Plant Game

## 📁 Estructura de carpetas

```
landing/assets/
├── plants/          ← diseños de la planta por etapa de evolución
├── bg/              ← fondo de la sala de cultivo
├── equipment/       ← iconos de equipamiento
└── FORMATO.md       ← este archivo
```

---

## 🌱 plants/ — Etapas de evolución (1 archivo por cada 10 niveles)

### Formato recomendado: PNG
- **Resolución base**: 20 × N píxeles (sin escalar)
- **Escala de render**: 6× (cada pixel = 6×6 px en pantalla)
- **Sin antialiasing / suavizado** — pixel art puro
- **Fondo transparente** (PNG con alpha)
- **Herramientas sugeridas**: Aseprite, Piskel, Libresprite

### Archivos a crear (uno por stage):

| Archivo                        | Nivel  | Tamaño base | Tamaño render |
|-------------------------------|--------|-------------|---------------|
| `stage_01_semilla.png`        | 1-10   | 20 × 17 px  | 120 × 102 px  |
| `stage_02_plantula.png`       | 11-20  | 20 × 15 px  | 120 × 90 px   |
| `stage_03_veg_temprana.png`   | 21-30  | 20 × 32 px  | 120 × 192 px  |
| `stage_04_veg_grande.png`     | 31-40  | 20 × 32 px  | 120 × 192 px  |
| `stage_05_preflora.png`       | 41-50  | 20 × 32 px  | 120 × 192 px  |
| `stage_06_flora.png`          | 51-60  | 20 × 32 px  | 120 × 192 px  |
| `stage_07_flora_media.png`    | 61-70  | 20 × 36 px  | 120 × 216 px  |
| `stage_08_cosecha.png`        | 71-80  | 20 × 38 px  | 120 × 228 px  |
| `stage_09_madura.png`         | 81-90  | 20 × 38 px  | 120 × 228 px  |
| `stage_10_legendaria.png`     | 91+    | 20 × 40 px  | 120 × 240 px  |

### Alternativa: SVG con pixel art vectorial
Si preferís SVG, usar `<rect>` elements en una grilla de 6×6 px.
Ejemplo: `<rect x="54" y="0" width="6" height="6" fill="#bef264"/>`

### Cómo integrar en el código:
Cuando tengas el PNG listo, en `game.html` reemplazar:
```javascript
// En EVOLUTION_STAGES, cambiar el grid por una URL:
{ minLvl:21, maxLvl:30, name:'VEG. TEMPRANA', emoji:'🪴', img:'assets/plants/stage_03_veg_temprana.png' }
```
Y en `renderPlant()` detectar si hay `img` y usar `<img>` en lugar de SVG `<rect>`.

---

## 🏠 bg/ — Fondo de sala de cultivo

### Formato: JPG o PNG
- **Resolución**: 420 × (alto de pantalla - 40px) ≈ 420 × 680 px
- **Relación de aspecto**: 9:16 aproximado (mobile portrait)
- Se aplica como `background-image` en `.room-bg`

### Archivo:
| Archivo              | Descripción                  |
|---------------------|------------------------------|
| `growroom.jpg`       | Vista frontal sala de cultivo |
| `growroom_dark.jpg`  | Versión oscura para stress   |

### Cómo integrar:
En game.html, CSS de `.room-bg`:
```css
.room-bg {
  background-image: url(assets/bg/growroom.jpg);
  background-size: cover;
  background-position: center;
}
```

---

## ⚙️ equipment/ — Iconos de dispositivos

### Formato: SVG (recomendado) o PNG
- **Tamaño**: 32 × 32 px (SVG: viewBox="0 0 32 32")
- **Estilo**: pixel art o icono flat simple
- **Colores sugeridos** por dispositivo:

| Archivo              | Dispositivo      | Color sugerido |
|---------------------|-----------------|----------------|
| `ac.svg`             | Aire acondicionado | `#38bdf8` azul frío |
| `humidificador.svg`  | Humidificador    | `#38bdf8` azul |
| `deshumidificador.svg`| Deshumidificador | `#4ade80` verde |
| `calefaccion.svg`    | Calefacción      | `#f97316` naranja |
| `extraccion.svg`     | Extracción       | `#fbbf24` amarillo |
| `intraccion.svg`     | Intracción       | `#fbbf24` amarillo |
| `ventilacion.svg`    | Ventilación      | `#a855f7` violeta |
| `riego.svg`          | Riego            | `#38bdf8` azul |
| `luces.svg`          | Luces LED        | `#ffffd0` blanco cálido |

### Cómo integrar:
En game.html, dentro de cada `.device-icon`:
```html
<div class="device-icon">
  <div class="dev-led s-on"></div>
  <img src="assets/equipment/ac.svg" width="24" height="24" alt="A/C">
</div>
```

---

## 🎨 Paleta de colores del juego

```
Verde saludable:   #22c55e / #4ade80
Rosa player:       #f472b6
Cyan stats:        #22d3ee
Violeta XP:        #a855f7
Naranja stress:    #f97316
Azul frío stress:  #38bdf8
Amarillo XP:       #fbbf24
Fondo oscuro:      #070710
Mylar carpa:       #d0d0d0
```
