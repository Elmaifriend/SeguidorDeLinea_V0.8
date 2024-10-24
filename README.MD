# Seguidor de Línea con Arduino Nano

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

## Ajuste de las Constantes del Algoritmo PID

El ajuste de las constantes **Kp**, **Ki** y **Kd** del algoritmo PID es crucial para garantizar un comportamiento estable y preciso del robot seguidor de línea. A continuación se describe cómo ajustar estas constantes paso a paso:

### Paso 1: Entender qué hace cada constante del PID

- **Kp (Proporcional)**: Controla la respuesta rápida del carrito. Si este valor es demasiado bajo, el robot reaccionará lentamente. Si es muy alto, el robot puede oscilar mucho alrededor de la línea.
- **Ki (Integral)**: Corrige el error acumulado con el tiempo. Si hay un pequeño error constante, este valor lo corregirá. Sin embargo, un valor de Ki demasiado alto hará que el sistema se vuelva inestable.
- **Kd (Derivativo)**: Reduce las oscilaciones. Un valor adecuado de Kd suaviza las correcciones, evitando que el robot "rebote" mientras sigue la línea.

### Paso 2: Ajustar Kp (Proporcional)

1. Comienza con **Ki = 0** y **Kd = 0** para simplificar el control solo al término proporcional.
2. Prueba con un valor bajo de **Kp**, como **0.1**.
3. Si el carrito no sigue bien la línea o se desvía demasiado, incrementa **Kp** poco a poco (0.1 por vez).
4. Continúa subiendo **Kp** hasta que el carrito siga la línea con ligeras oscilaciones. Si el valor de **Kp** es muy alto, el carrito oscilará rápidamente de un lado a otro. Si esto ocurre, baja el valor de **Kp**.

### Paso 3: Ajustar Kd (Derivativo)

1. Mantén el valor de **Kp** que encontraste en el paso anterior y deja **Ki = 0**.
2. Comienza a aumentar **Kd**, probando con un valor pequeño, como **0.05**.
3. **Kd** ayuda a amortiguar las oscilaciones. Si el carrito oscila mucho, incrementa **Kd** lentamente.
4. Ajusta **Kd** hasta que las oscilaciones se reduzcan considerablemente. Si **Kd** es demasiado alto, el carrito será lento para reaccionar a los cambios en la línea, en ese caso, reduce **Kd**.

### Paso 4: Ajustar Ki (Integral)

1. Ajusta **Ki** solo si el carrito sigue la línea pero no se mantiene centrado.
2. Si el carrito tiende a desviarse lentamente de la línea, es probable que necesites ajustar **Ki**.
3. Aumenta **Ki** en pequeñas cantidades, como **0.01**.
4. Ajusta **Ki** hasta que el carrito siga la línea sin desviarse con el tiempo. Un valor de **Ki** muy alto puede causar inestabilidad, así que procede con precaución.

### Paso 5: Ajuste Fino y Pruebas

1. Una vez que tengas valores razonables para **Kp**, **Ki**, y **Kd**, realiza ajustes pequeños a cada constante para afinar la respuesta del carrito.
2. Prueba el carrito en diferentes condiciones (curvas, tramos rectos, etc.) para asegurarte de que funcione bien en todas las situaciones.

### Ejemplo de Valores PID

A continuación se muestra un ejemplo de valores PID típicos para un robot seguidor de línea. Estos valores pueden variar según tu configuración específica:

- **Kp** = 0.2
- **Ki** = 0.01
- **Kd** = 0.1

### Resumen de Pasos:

1. Establece **Ki = 0**, **Kd = 0** y ajusta **Kp** hasta obtener un seguimiento con pequeñas oscilaciones.
2. Ajusta **Kd** para reducir las oscilaciones.
3. Ajusta **Ki** solo si el carrito no se mantiene centrado en la línea.
4. Realiza ajustes finos para optimizar el comportamiento del carrito.

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
