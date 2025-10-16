# Taxímetro Digital con STM32 Nucleo-F411RE

Este repositorio contiene el firmware para un taxímetro digital implementado en una placa de desarrollo ST Nucleo-F411RE. El proyecto es una demostración práctica del uso de periféricos de hardware como Timers e Interrupciones Externas para crear un sistema embebido robusto, eficiente y con alta capacidad de respuesta.

!

## 📜 Descripción del Proyecto

El sistema muestra un contador numérico en un display de 4 dígitos y 7 segmentos. El valor del contador, que opera en un rango de **1 a 4095**, se controla mediante un encoder rotativo. Adicionalmente, un pulsador permite al usuario cambiar la tasa de refresco (FPS) del display en tiempo real, demostrando cómo modificar el comportamiento del sistema de forma interactiva.

El diseño del software se centra en las buenas prácticas para sistemas embebidos, delegando tareas repetitivas y de alta frecuencia al hardware del microcontrolador. Este enfoque libera a la CPU y garantiza una respuesta instantánea a las interacciones del usuario.

***

## ✨ Características Principales

* **Contador Circular:** El contador opera entre 1 y 4095 con comportamiento *wrap-around*, es decir, al pasar de 4095 vuelve a 1, y al bajar de 1 salta a 4095.
* **Control por Encoder Asistido por Hardware:** La lectura del encoder rotativo se gestiona íntegramente por el periférico **TIM2** en modo Encoder, asegurando que no se pierda ningún paso y liberando a la CPU de esta tarea.
* **Display Multiplexado por Interrupción:** El control del display de 4 dígitos se realiza mediante multiplexado por tiempo, gestionado por una interrupción del **TIM3**. Esto garantiza un refresco visual constante y sin parpadeos.
* **Control Dinámico de FPS:** El usuario puede ciclar entre diferentes velocidades de refresco (**5, 10, 20, 30 y 60 FPS**) para el display.
* **Respuesta Instantánea del Botón:** El pulsador se gestiona mediante una **Interrupción Externa (EXTI)**, garantizando que su acción sea reconocida de forma inmediata, sin retardos ni dependencia de la carga de trabajo del sistema.

***

## ⚙️ Hardware Requerido

* **Placa de Desarrollo:** ST Nucleo-F411RE
* **Display:** 1x Display de 4 dígitos y 7 segmentos (Ánodo Común)
* **Control de Dígitos:** 4x Transistores PNP (ej. 2N3906)
* **Encoder:** 1x Encoder rotativo incremental
* **Pulsador:** 1x Pulsador táctil
* **Resistencias:**
    * 7x 220Ω (para los segmentos A-G)
    * 4x 1kΩ (para las bases de los transistores PNP)
    * 1x 10kΩ (resistencia de pull-down para el pulsador)

***

## 🛠️ Software y Herramientas

* **IDE:** STM32CubeIDE
* **Lenguaje:** C
* **Bibliotecas:** STM32Cube HAL (Hardware Abstraction Layer)

***

## 🔌 Diagrama de Conexiones (Pinout)

| Componente              | Pin del Componente              | Conexión en Nucleo-F411RE |
| :---------------------- | :------------------------------ | :------------------------ |
| **Display** | Segmento A                      | `PB0`                     |
|                         | Segmento B                      | `PB7`                     |
|                         | Segmento C                      | `PB14`                    |
|                         | Segmento D                      | `PB15`                    |
|                         | Segmento E                      | `PB1`                     |
|                         | Segmento F                      | `PB2`                     |
|                         | Segmento G                      | `PB13`                    |
| **Control de Dígitos** | Base Transistor D1 (Millares)   | `PA8`                     |
|                         | Base Transistor D2 (Centenas)   | `PA7`                     |
|                         | Base Transistor D3 (Decenas)    | `PA6`                     |
|                         | Base Transistor D4 (Unidades)   | `PA10`                    |
| **Encoder Rotativo** | CLK (Terminal A)                | `PA0`                     |
|                         | DT (Terminal B)                 | `PA1`                     |
| **Pulsador FPS** | Un terminal                     | `PA5`                     |

***
## Esquemáticos
<img width="1143" height="883" alt="image" src="https://github.com/user-attachments/assets/611a64b7-dcdd-4459-94e3-23d8d36701b9" />
<img width="1137" height="883" alt="image" src="https://github.com/user-attachments/assets/a2a5847b-70f7-418e-9b0c-439ea316ebe7" />




## 🚀 Cómo Usar

1.  **Clonar el Repositorio:** Clona este repositorio y ábrelo con STM32CubeIDE.
2.  **Compilar y Cargar:** Compila el proyecto y carga el firmware en la placa Nucleo-F411RE.
3.  **Controlar el Contador:** Gira el encoder rotativo para incrementar o decrementar el valor mostrado en el display.
4.  **Cambiar FPS:** Presiona el pulsador para cambiar la velocidad de refresco del display. La nueva velocidad se aplicará de forma instantánea.
