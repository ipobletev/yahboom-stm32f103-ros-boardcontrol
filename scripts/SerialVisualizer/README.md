# Yahboom STM32 Serial Visualizer

Este es un visualizador en tiempo real basado en Python y Qt para monitorizar y controlar la placa Yahboom STM32F103 a través del protocolo `SerialROS`.

## 🚀 Inicio Rápido

### Windows
Simplemente ejecuta el archivo batch:
```cmd
.\launch.bat
```

### Linux / macOS
Da permisos de ejecución y lanza el script:
```bash
chmod +x launch.sh
./launch.sh
```

Los scripts de lanzamiento crearán automáticamente un entorno virtual (`.venv`), instalarán las dependencias necesarias y abrirán la aplicación.

## 📊 Características

- **Dashboard de Estado**: Visualiza en tiempo real el estado del sistema (IDLE, MOVING, E_STOP), modo de operación y detección de movimiento.
- **Gráficos en Tiempo Real**: Monitoreo de aceleración (m/s²) y velocidad angular (deg/s) del IMU.
- **Lectura de Encoders**: Visualización de los valores acumulados de los 4 motores (FL, FR, BL, BR).
- **Control Remoto**:
  - Envío de comandos de velocidad (`cmd_vel`).
  - Cambio entre modo Manual y Autónomo.
  - Botón de Parada de Emergencia (E-Stop).

## 🛠️ Estructura del Proyecto

- `visualizer.py`: Aplicación principal con la interfaz Qt (PySide6).
- `serial_ros.py`: Implementación del protocolo de comunicación (packing/unpacking/checksum).
- `simulator.py`: Script de utilidad para simular datos de la placa (útil para desarrollo sin hardware).
- `requirements.txt`: Dependencias de Python.

## 🧪 Pruebas sin Hardware

Si no tienes la placa conectada, puedes usar el simulador:

1. Crea un par de puertos serie virtuales (ej. `COM10` <-> `COM11` en Windows con com0com, o `pts` en Linux con socat).
2. Ejecuta el simulador en un puerto:
   ```bash
   python simulator.py COM10
   ```
3. Lanza el visualizador, selecciona `COM11` y pulsa **Connect**.

## 📦 Dependencias Principales

- **PySide6**: Framework para la interfaz gráfica.
- **pyqtgraph**: Librería de alto rendimiento para gráficos 2D.
- **pyserial**: Comunicación a través del puerto serie.
- **numpy**: Procesamiento de datos numéricos.
