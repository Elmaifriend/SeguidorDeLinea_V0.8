# Proyecto: Seguidor de Línea con Arduino Nano

Este proyecto implementa un robot seguidor de línea utilizando un Arduino Nano y diversas librerías para controlar sensores y motores.

## Dependencias

Este proyecto requiere las siguientes librerías:

- [PID](https://github.com/br3ttb/Arduino-PID-Library): Para el control PID.
- [SparkFun_TB6612](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library): Para el control del controlador de motores TB6612FNG.
- [QTRSensors](https://github.com/pololu/qtr-sensors-arduino): Para la lectura de la matriz de sensores QTR.
- [JC_Button](https://github.com/JChristensen/JC_Button): Para el manejo de botones.

Puedes instalar estas librerías directamente desde el gestor de bibliotecas del IDE de Arduino.

## Lista de Pines

### Sensor Array QTR8A | Arduino Nano

| Sensor QTR | Pin Arduino |
|------------|-------------|
| D1         | A0          |
| D2         | A1          |
| D3         | A2          |
| D4         | A3          |
| D5         | A4          |
| D6         | A5          |
| D7         | A6          |
| D8         | A7          |

### TB6612FNG (Controlador de Motores) | Arduino Nano

| Componente | Pin Arduino |
|------------|-------------|
| AIN2       | D10         |
| AIN1       | D9          |
| BIN1       | D6          |
| BIN2       | D5          |
| PWMA       | D11         |
| PWMB       | D3          |
| STBY       | GND         |

### Botones | Arduino Nano

| Botón | Pin Arduino |
|-------|-------------|
| BTN1  | D2          |
| BTN2  | D4          |

## ¿Cómo usar el robot?

1. **Calibración**: 
   - Coloca el robot sobre la pista (una línea negra sobre fondo blanco).
   - Presiona el botón **BTN1** para iniciar el proceso de calibración.
   - Mueve el robot suavemente sobre la pista para que los sensores puedan detectar tanto la línea negra como el fondo blanco. La calibración durará unos 8 segundos.
   
2. **Seguir la línea**:
   - Presiona el botón **BTN2** para comenzar la rutina de seguimiento de línea. El robot esperará 3 segundos y luego comenzará a seguir la línea negra automáticamente.

## Ajuste de Constantes del Algoritmo PID

El ajuste de las constantes **Kp**, **Ki** y **Kd** del algoritmo PID es crucial para garantizar un comportamiento estable y preciso del robot seguidor de línea. Aquí te presento un método sencillo para ajustar estas constantes:

1. **Ajuste del Kp (Proporcional)**:
   - Comienza aumentando **Kp** desde un valor bajo hasta que el robot siga la línea, pero con ligeras oscilaciones.
   - Si el valor es demasiado alto, el robot oscilará mucho; si es muy bajo, no corregirá lo suficientemente rápido.

2. **Ajuste del Ki (Integral)**:
   - Establece **Ki** en 0 al principio.
   - Una vez que tengas un valor **Kp** razonable, ajusta **Ki** ligeramente para corregir cualquier error acumulado (por ejemplo, si el robot se desvía gradualmente).
   - Si el valor de **Ki** es demasiado alto, puede causar inestabilidad y oscilaciones más grandes.

3. **Ajuste del Kd (Derivativo)**:
   - Después de ajustar **Kp** y **Ki**, comienza a aumentar **Kd** para reducir las oscilaciones.
   - **Kd** ayuda a amortiguar el sistema, permitiendo que las correcciones sean suaves en lugar de bruscas.
   - Ajusta **Kd** hasta que las oscilaciones se minimicen sin sacrificar la capacidad de respuesta.

Este proceso se conoce como **método de prueba y error** y es el más común para ajustar las constantes PID en proyectos de este tipo.

## Explicación de Funciones

### `inicializar()`
Esta función inicializa todos los componentes necesarios para el robot:
- Configura los botones y los motores.
- Inicializa la matriz de sensores QTR para la detección de la línea.

### `calibrar()`
Este método realiza la calibración de los sensores QTR:
- Durante un ciclo de 8 segundos, lee los valores de los sensores y ajusta los valores mínimos y máximos de luz reflejada.
- Los valores obtenidos permiten normalizar las lecturas futuras, lo que mejora la precisión del seguimiento de línea.

### `seguirLinea()`
Este método ejecuta el algoritmo principal de seguimiento de línea:
- Lee los sensores utilizando `qtr.readLine()`, que devuelve la posición de la línea en relación con la matriz de sensores.
- Calcula el error comparando la posición de la línea con el centro de la matriz de sensores.
- Utiliza un controlador PID para ajustar la velocidad de los motores, corrigiendo la trayectoria del robot y asegurando que siga la línea de manera fluida.

### `ejecutar()`
Este método contiene el ciclo principal del programa:
- Espera la entrada del usuario presionando los botones. 
- Si el botón **BTN1** es presionado, inicia el proceso de calibración.
- Si el botón **BTN2** es presionado, espera 3 segundos y luego comienza el seguimiento de la línea.
