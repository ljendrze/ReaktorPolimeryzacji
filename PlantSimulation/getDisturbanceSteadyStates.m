% Wartości parametrów okresląjace punkt równowagi reaktora polimeryzacji 
% opisywanego w artykule. Zmienne stanu, wartość sterowania i zakłócenie
% w postaci różnicy w stosunku do strumienia dopływającego.

addpath('../PlantData');
load( 'reactorData.mat' );

% Czas symulacji obiektu.
time = [ 0 10 ]';

% Parametry krańców przedziałów symulacji i rozdzielczości symulacji.
zStarting = -0.2;
zFinal = 0.2;
step = 0.001;

dataSize = (zFinal - zStarting) / step;

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć 
% wartość tolerancji bezwzględnej i względnej metody ode45, 
% aby na wyjściu obiektu ustalał się stan (a nie były widoczne 
% oscylacje w stanie ustalonym).
opts = odeset('AbsTol',1e-8,'RelTol',1e-10);

z = zStarting;
u = u0;
outputSS = zeros(dataSize,1);
disturbanceSS = zeros(dataSize,1);
i = 1;

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ==================== %
simulationStep = 0;                                                    %
lettersWritten = 0;                                                    %
finishTime = dataSize;                                                 %
fprintf('\n');                                                         %
% ==================================================================== %

while z <= zFinal

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   if i > simulationStep*(finishTime/100)                              %
      while lettersWritten > 0                                         %
         fprintf('\b');                                                %
         lettersWritten = lettersWritten - 1;                          %
      end                                                              %
      lettersWritten = fprintf('Progress:    %d%%',simulationStep);    %
      simulationStep = simulationStep + 1;                             %
   end                                                                 %
   % ================================================================= %

   [ tout, xout ] = ode45( @plantFunction, time, x0, opts, u, z);
   x = xout( length(xout), : );
   outputSS(i) = x(4) / x(3);
   disturbanceSS(i) = z;
   i = i + 1;
   z = z + step;
end

plot( disturbanceSS, outputSS );
grid on;
ylabel('Wyjscie obiektu');
xlabel('Wartosc zaklocenia');

save( '../PlantData/disturbanceSteadyStates.mat', 'disturbanceSS', 'outputSS' );

fprintf('\n\n');

rmpath('../PlantData');
