% Cargar la imagen
%image = imread('LHC_cross_section.jpg');
% image = imread('LCH_map_cross_section.mat');
% image = imread('CCC_ROI.jpg');
load("LCH_map_cross_section.mat")
show(occupancyMap)

% Convertir la imagen a formato double para hacer cálculos
image = im2double(image);

% Definir el umbral de claridad. Los colores cercanos a blanco (RGB cercano a [1, 1, 1]) son libres
threshold = 0.95;  % Ajusta este valor si necesitas más precisión

% Crear un mapa binario donde 1 es libre (blanco o casi blanco) y 0 es obstáculo (otro color)
% Aquí comparamos los canales de color RGB con el umbral
binaryMap = all(image < threshold, 3);  % Verifica que R, G, y B sean mayores que el umbral

% Redimensionar la imagen a un grid de 45x45
gridMap = imresize(binaryMap, [90, 90]);

% Crear un mapa de ocupación con las dimensiones del grid (45x45)
occupancyMap = binaryOccupancyMap(90, 90, 1);  % Celdas de 1 metro de lado, por ejemplo

% Ajustar los valores de ocupación del mapa
setOccupancy(occupancyMap, gridMap);

% Mostrar el mapa de ocupación
figure;
%show(occupancyMap);
title('Mapa de ocupación generado a partir de una imagen');

hold on;
for i = 0:90
    plot([i i], [0 90], 'k'); % Dibujar línea de la cuadrícula en el eje X
    plot([0 90], [i i], 'k'); % Dibujar línea de la cuadrícula en el eje Y
end

%save("LCH_map_cross_section","occupancyMap");
%save("Room","occupancyMap");

%Funtion to paint the map
cellStates=flipud(getOccupancy(occupancyMap));

% cellStates(cellStates == 1) = 5;
% cellStates(cellStates == false) = 2;
cellStates=cellStates*2+2;
cellStates(75,75)=0; 
cellStates(9,73)=5;
paintMap(cellStates, occupancyMap)

%save("CCC_ROI_CellStates","cellStates");
function paintMap(cellStates, map)
    cla;  % Limpiar el grÃ¡fico anterior
    % Volver a dibujar el mapa sin borrar las celdas coloreadas
    %show(map);
    hold on;
    % Volver a dibujar la cuadrÃ­cula manualmente
    % Redibujar las celdas ya exploradas y las celdas frontera
    for x = 1:90
        for y = 1:90
            if cellStates(y, x) == 1
                % Celda ocupada (obstÃ¡culo)
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'r', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 0
                % Celda libre
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'g', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 3
                 % Celda frontera (morado)
                 fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'm', 'FaceAlpha', 0.3);
            elseif cellStates(y, x) == 5
                % Celda objetivo
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'b', 'FaceAlpha', 0.7);
            elseif cellStates(y, x) == 4
                % zona probable
                fill([x-1, x, x, x-1], [y-1, y-1, y, y], 'y', 'FaceAlpha', 0.3);
            end
        end
    end
    for i = 0:90
        plot([i i], [0 90], 'k'); % Dibujar lÃ­nea de la cuadrÃ­cula en el eje X
        plot([0 90], [i i], 'k'); % Dibujar lÃ­nea de la cuadrÃ­cula en el eje Y
    end
end