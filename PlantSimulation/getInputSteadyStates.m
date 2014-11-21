% Wartości parametrów okresląjace punkt równowagi reaktora polimeryzacji 
% opisywanego w artykule. Zmienne stanu, wartość sterowania i zakłócenie
% w postaci różnicy w stosunku do strumienia dopływającego.

addpath('../PlantData');
load( 'reactorData.mat' );

% Czas symulacji obiektu.
time = [ 0 10 ]';

% Parametry krańców przedziałów symulacji i rozdzielczości symulacji.
uStarting = 0.001;
uFinal = 0.2;
step = 0.0001;

dataSize = (uFinal - uStarting) / step;

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć 
% wartość tolerancji bezwzględnej i względnej metody ode45, 
% aby na wyjściu obiektu ustalał się stan (a nie były widoczne 
% oscylacje w stanie ustalonym).
opts = odeset('AbsTol',1e-8,'RelTol',1e-10);

u = uStarting;
z = z0;
x = x0;

outputSS = zeros(dataSize,1);
inputSS = zeros(dataSize,1);
i = 1;

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ==================== %
simulationStep = 0;                                                    %
lettersWritten = 0;                                                    %
finishingTime = dataSize;                                              %
fprintf('\n');                                                         %
% ==================================================================== %

while u <= uFinal

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   if i > simulationStep*(finishingTime/100)                           %
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
   inputSS(i) = u;
   i = i + 1;
   u = u + step;
end

plot( inputSS, outputSS );
grid on;
ylabel('Wyjscie obiektu');
xlabel('Wartosc wejscia');

save( '../PlantData/inputSteadyStates.mat', 'inputSS', 'outputSS' );

fprintf('\n\n');

rmpath('../PlantData');
