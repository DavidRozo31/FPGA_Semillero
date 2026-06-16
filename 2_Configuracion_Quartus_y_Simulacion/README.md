# Configuración Quartus y Simulación

Lo primero que debemos de hacer es instalar Quartus desde el siguiente link (18.1) :
https://www.altera.com/downloads/fpga-development-tools/quartus-prime-lite-edition-design-software-version-18-1-windows

Posteriormente al abrir Quartus nos encontraremos con la interfaz de inicio, y buscaremos donde dice "New Proyect Wizard"

<img width="1679" height="993" alt="image" src="https://github.com/user-attachments/assets/004db07c-4e98-4653-af79-4bcc88dbfaf4" />

Posteriormente colocamos donde queremos que se ubique el archivo y el nombre del proyecto (Por default Quartus coloca al archivo top level el mismo nombre del proyecto, pero esto no es estrictamente necesario), Yo recomiendo crear una carpeta llamada como el proyecto y dentro de esta ubicarlo.

<img width="791" height="624" alt="image" src="https://github.com/user-attachments/assets/17621fbb-2624-4d59-9d1b-da355233eced" />

Damos "Next" 3 veces y en "Family, Device & Settings" colocamos nuestro modelo de FPGA Cyclone IV Core 8 (O la que tengamos nosotros) y la seleccionamos en la parte de abajo, como se muestra en la imagen con sombreado azul

<img width="789" height="624" alt="image" src="https://github.com/user-attachments/assets/f4d5c4dc-164e-4187-a96a-095928dc19f4" />

Luego en "EDA tools Settings" colocamos en Simulation "ModelSim - Altera" y "VHDL" como se evidencia en la imagen:

<img width="795" height="622" alt="image" src="https://github.com/user-attachments/assets/0bdacdef-10e4-4271-8889-2d6d5daddfcb" />

Damos click en Finish y ya tenemos nuestro proyecto creado en Quartus, si deseamos ver los archivos, vamos a la parte izquierda y vemos donde dice Hierarchy, y lo cambiamos por Files, adicionalmente le damos a la hoja blanca del panel superior para crear nuestro primer archivo VHDL

<img width="665" height="658" alt="image" src="https://github.com/user-attachments/assets/c3885c74-84a7-46ee-9400-3d09cc64ef92" />

En dicho archivo podemos colocar un ejemplo simple de una compuerta AND que esta representada con el siguiente codigo: 







