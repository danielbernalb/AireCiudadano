# Guía de Reestructuración del Código - AireCiudadano

## 📋 ¿Qué cambió?

El archivo `main.cpp` de 9100 líneas fue reestructurado en módulos organizados por funcionalidad. Esto hace que el código sea:
- ✅ **Más fácil de entender** - Cada archivo tiene una responsabilidad clara
- ✅ **Más fácil de mantener** - Los cambios se hacen en un solo lugar
- ✅ **Más fácil de depurar** - Los errores se aíslan rápidamente
- ✅ **Colaborativo** - Voluntarios con nivel básico pueden contribuir

## 📁 Nueva Estructura de Archivos

```
src/
├── main.cpp              # Archivo principal (ahora más pequeño)
├── main.hpp              # Encabezado principal actualizado
│
├── config/               # ⚙️ CONFIGURACIÓN (¡MODIFICA AQUÍ!)
│   ├── config.h          # Flags y definiciones (true/false)
│   └── types.h           # Variables globales y estructuras
│
├── wifi/                 # 📡 COMUNICACIÓN WIFI
│   └── wifi.h            # Funciones WiFi y portal cautivo
│
├── mqtt/                 # ☁️ COMUNICACIÓN MQTT
│   └── mqtt.h            # Funciones MQTT (nube)
│
├── sensors/              # 🔬 SENSORES
│   └── sensors.h         # Funciones para todos los sensores
│
├── display/              # 🖥️ PANTALLAS
│   └── display.h         # Funciones para OLED/TTGO
│
├── storage/              # 💾 ALMACENAMIENTO
│   └── storage.h         # EEPROM, SD, RTC
│
├── bluetooth/            # 📱 BLUETOOTH
│   └── bluetooth.h       # Funciones Bluetooth BLE
│
├── mobile/               # 📶 DATOS MÓVILES (SIM)
│   └── mobile.h          # Funciones GSM/LTE
│
└── utils/                # 🛠️ UTILIDADES
    └── utils.h           # Funciones generales
```

## 🚀 ¿Por dónde empezar?

### Para voluntarios nuevos:

1. **Configurar tu sensor** → Edita `src/config/config.h`
   - Activa/desactiva componentes (WiFi, sensores, pantalla)
   - Solo cambia `true` por `false` según tu hardware

2. **Entender las funciones** → Lee los archivos `.h`
   - Cada archivo tiene comentarios explicativos
   - Las funciones están organizadas por categoría

3. **Modificar comportamiento** → Busca la función relevante
   - Ejemplo: ¿Quieres cambiar cómo se muestra PM2.5? → `src/display/display.h`
   - Ejemplo: ¿Quieres agregar un sensor? → `src/sensors/sensors.h`

## 📖 Convenciones de Nombres

| Prefijo | Significado | Ejemplo |
|---------|-------------|---------|
| `Setup_` | Inicializa un componente | `Setup_Sensor()` |
| `Read_` | Lee medición de sensor | `Read_Sensor()` |
| `Connect_` | Establece conexión | `Connect_WiFi()` |
| `Send_` | Envía datos | `Send_Message_Cloud_App_MQTT()` |
| `Print_` | Imprime información | `Print_Config()` |
| `Update_` | Actualiza display/datos | `Update_Display()` |
| `display` | Funciones de visualización | `displaySensorData()` |

## 🔧 Flujo de Trabajo Típico

### Agregar un nuevo sensor:

1. Abre `src/config/config.h` y agrega:
```cpp
#define MiNuevoSensor false  // true para activar
```

2. Abre `src/sensors/sensors.h` y agrega las funciones:
```cpp
void Setup_MiNuevoSensor();
void Read_MiNuevoSensor();
```

3. Implementa las funciones en `main.cpp` (o crea un archivo .cpp nuevo)

### Cambiar intervalo de publicación MQTT:

1. Busca en `src/config/types.h`:
```cpp
extern unsigned int MQTT_loop_review_duration;
```

2. Modifica el valor en `main.cpp` o agrega configuración en `config.h`

## 📝 Reglas de Oro

1. **NO modifiques variables en los archivos `.h`** - Solo declara funciones
2. **SIEMPRE lee los comentarios** - Explican qué hace cada función
3. **Prueba cambios pequeños** - No hagas muchos cambios a la vez
4. **Usa el Serial Monitor** - Imprime mensajes para debugging
5. **Respeta la estructura** - Mantén las funciones en su módulo correspondiente

## 🆘 Solución de Problemas Comunes

### "No compila" / Errores de definición
- Verifica que todos los `#include` estén presentes
- Asegúrate de que las funciones estén declaradas en los `.h`

### "El sensor no funciona"
- Revisa `src/config/config.h` - ¿Activaste el sensor correcto?
- Verifica `Test_Sensor()` en `src/sensors/sensors.h`

### "No se conecta a WiFi"
- Revisa credenciales en portal cautivo
- Verifica `Connect_WiFi()` en `src/wifi/wifi.h`

## 📞 ¿Necesitas ayuda?

1. Lee los comentarios en los archivos `.h` - ¡Están ahí para ayudarte!
2. Revisa el `README.md` principal del proyecto
3. Consulta con otros voluntarios en el canal de comunicación

## ✨ Próximos Pasos

Esta reestructuración es el primer paso. En el futuro:
- [ ] Mover implementaciones de `main.cpp` a archivos `.cpp` separados
- [ ] Agregar más comentarios en español
- [ ] Crear ejemplos de configuración para casos comunes
- [ ] Documentar cada función con ejemplos de uso

---

**¡Gracias por contribuir a AireCiudadano!** 🌍💚

Cada línea de código limpio ayuda a medir mejor la calidad del aire.
