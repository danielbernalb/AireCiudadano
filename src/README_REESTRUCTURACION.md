# 📁 Reestructuración del Código - AireCiudadano

## ¿Qué hicimos?

Transformamos el archivo `main.cpp` de **9100 líneas** en un proyecto modular organizado por funcionalidades.

## 🎯 Beneficios para Voluntarios

### Antes (main.cpp gigante):
- ❌ 9100 líneas en un solo archivo
- ❌ Difícil encontrar funciones específicas
- ❌ Cambios pequeños requerían revisar todo el archivo
- ❌ Intimidante para programadores básicos

### Ahora (módulos organizados):
- ✅ Cada módulo tiene < 200 líneas de declaraciones
- ✅ Funciones agrupadas por categoría (WiFi, sensores, display, etc.)
- ✅ Comentarios explicativos en cada función
- ✅ Fácil de navegar y entender

## 📂 Estructura de Carpetas

```
src/
├── main.cpp              # Archivo principal (setup y loop)
├── main.hpp              # Incluye todos los módulos
│
├── config/               # ⚙️ CONFIGURACIÓN - ¡EMPIEZA AQUÍ!
│   ├── config.h          # Flags true/false para activar componentes
│   └── types.h           # Variables globales del sistema
│
├── wifi/                 # 📡 WiFi y portal cautivo
│   └── wifi.h            # Conexión WiFi, servidor web
│
├── mqtt/                 # ☁️ Comunicación con la nube
│   └── mqtt.h            # Publicación MQTT, reconexión
│
├── sensors/              # 🔬 Sensores de partículas y ambientales
│   └── sensors.h         # Setup y lectura de sensores
│
├── display/              # 🖥️ Pantallas OLED y TTGO
│   └── display.h         # Mostrar datos en pantalla
│
├── storage/              # 💾 Guardado de datos
│   └── storage.h         # EEPROM, SD card, RTC
│
├── bluetooth/            # 📱 Bluetooth BLE
│   └── bluetooth.h       # Comunicación con celulares
│
├── mobile/               # 📶 Datos móviles (SIM card)
│   └── mobile.h          # Conexión 3G/4G
│
└── utils/                # 🛠️ Funciones utilitarias
    └── utils.h           # Utilidades generales
```

## 🚀 Guía Rápida para Voluntarios

### 1. Configurar tu dispositivo

Edita `src/config/config.h`:

```cpp
// Activar WiFi
#define Wifi true        // Cambia a false si no usas WiFi

// Activar sensor de partículas
#define TwoPMS false     // true = 2 sensores, false = 1 sensor

// Activar pantalla
#define OLED96display true  // true si tienes pantalla OLED 0.96"

// Activar medición de sonido
#define SoundMeter true  // true para medidor de ruido
```

### 2. Entender qué hace cada función

Lee los archivos `.h`. Ejemplo: `src/sensors/sensors.h`:

```cpp
/**
 * @brief Lee medición de sensor de CO2
 * 
 * Actualiza variables globales de CO2, temperatura y humedad
 */
void Read_CO2sensor();
```

Los comentarios explican:
- **Qué hace** la función
- **Cuándo usarla**
- **Qué valores modifica**

### 3. Agregar nueva funcionalidad

**Ejemplo: Agregar sensor de temperatura extra**

Paso 1: Agrega flag en `config.h`:
```cpp
#define TempSensorExtra false  // true para activar
```

Paso 2: Declara funciones en `sensors.h`:
```cpp
void Setup_TempSensorExtra();
void Read_TempSensorExtra();
```

Paso 3: Implementa en `main.cpp` o crea `sensors_temp.cpp`

## 📖 Convenciones de Nombres

| Prefijo | Propósito | Ejemplo |
|---------|-----------|---------|
| `Setup_` | Inicializar hardware | `Setup_Sensor()` |
| `Read_` | Leer mediciones | `Read_Sensor()` |
| `Connect_` | Establecer conexión | `Connect_WiFi()` |
| `Send_` | Enviar datos | `Send_Message_Cloud_App_MQTT()` |
| `Print_` | Mostrar información | `Print_Config()` |
| `Update_` | Actualizar display | `Update_Display()` |
| `display` | Funciones de pantalla | `displaySensorData()` |

## 🔍 ¿Dónde buscar?

| Quieres modificar... | Busca en... |
|---------------------|-------------|
| Activar/desactivar componentes | `config/config.h` |
| Conexión WiFi | `wifi/wifi.h` |
| Envío de datos a la nube | `mqtt/mqtt.h` |
| Lectura de sensores | `sensors/sensors.h` |
| Mostrar en pantalla | `display/display.h` |
| Guardar en SD/EEPROM | `storage/storage.h` |
| Bluetooth | `bluetooth/bluetooth.h` |
| Datos móviles (SIM) | `mobile/mobile.h` |
| Funciones generales | `utils/utils.h` |

## 💡 Consejos para Principiantes

1. **Empieza pequeño**: Cambia solo un `true/false` en `config.h`
2. **Lee comentarios**: Cada función tiene explicación en español
3. **Usa Serial.print()**: Agrega mensajes para ver qué hace tu código
4. **Prueba frecuentemente**: Compila después de cada cambio pequeño
5. **No tengas miedo**: Los archivos `.h` solo declaran, no implementan

## ❓ Preguntas Frecuentes

### ¿Puedo borrar algún archivo .h?
**NO**. Cada archivo es necesario para compilar.

### ¿Dónde está el código real de las funciones?
En `main.cpp` (por ahora). Los `.h` solo dicen qué funciones existen.

### ¿Cómo sé qué variables puedo usar?
Revisa `config/types.h` - ahí están todas las variables globales.

### ¿Puedo crear mis propios archivos .h?
¡Sí! Crea una carpeta nueva en `src/` y sigue el mismo patrón.

## 🎓 Próximos Pasos (Futuro)

Esta es la **Fase 1** de reestructuración. En el futuro:

- [ ] Mover el código de las funciones a archivos `.cpp` separados
- [ ] Reducir `main.cpp` a menos de 500 líneas
- [ ] Agregar ejemplos de configuración para cada caso de uso
- [ ] Crear tests automáticos para cada módulo

---

**¿Listo para contribuir?** 🚀

Empieza por leer `config/config.h` y entender las opciones disponibles.

¡Cualquier mejora, por pequeña que sea, ayuda al proyecto! 💚
