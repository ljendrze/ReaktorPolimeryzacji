% Skrypt realizujący sterowanie reaktorem polimeryzacji za pomocą algorytmu
% regulacji predykcyjnej DMC. W wersji podstawowej wykorzystywany jest
% model liniowy obiektu, a algorytm zaimplementowany został w wersji
% numerycznej.

addpath('../CustomToolbox/DMC');
addpath('../PlantData');
addpath('../Models');

% Wartości parametrów określąjace punkt równowagi reaktora polimeryzacji 
% opisywanego w artykule ( u0, x0, y0, z0 ).
load('reactorData.mat');
load('linearModel.mat');

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć wartość
% tolerancji bezwzględnej i względnej metody ode45, aby na wyjściu obiektu
% ustalał się stan (a nie były widoczne oscylacje w stanie ustalonym).
opts=odeset('AbsTol', 1e-6, 'RelTol', 1e-8);

% Długości horyzontów:
D = 80;
N = 30;
Nu = 1;

% Przygotowanie wymaganych macierzy Psi, Lambda, Mp i M:
lambda = 1e12;
Lambda = lambda*eye(Nu);

psi = 1;
Psi = psi*eye(N);

% % Aby ustalić dobrą wartość parametru kary za przyrost sterowania, dobrze
% % jest porównać wartości znajdujące się na diagonali macierzy [M' * M],
% % ponieważ do obliczania wzmocnień w algorytmie DMC do elementów 
% % na przekątnej tej macierzy dodajemy macierz diagonalną Lambda
% % (Lambda = lambda*eye).
% diagonalM = zeros(1,Nu);
% MProduct = M' * M;
% for i = 1 : Nu
%    diagonalM(i) = MProduct(i,i);
% end

% Skracanie czasu symulacji, dla przejrzystszych wykresów.
time = [0:1/60:2];

simulationLength = size(time,2);

% Stworzenie i obliczenie wektora wartości zadanej.
outputDesiredTrajectory = y0 * ones(1, simulationLength);

for i = 10 : simulationLength
   outputDesiredTrajectory(i) = 30000;
end

disturbanceTrajectory = zeros(simulationLength,1);
for i = 70 : simulationLength
   disturbanceTrajectory(i) = 0;
end


% Przygotowanie wektorów sterowania i wartości. Wektory te będą
% wykorzystane do rysowania wykresów schodkowych.
plotOutput = zeros( simulationLength, 1 );
plotControl = zeros( simulationLength, 1 );
 
Tp = 1/60;

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulation_step = 0;                                                %
letters_written = 0;                                                %
finish_time = simulationLength;                                     %
fprintf('\n');                                                      %
% ================================================================= %

quadprog_options = optimset('Algorithm','active-set','Display','off');

DMCObject = struct( ...
   'model', linearModel, ...
   'D', D, ...
   'N', N, ...
   'Nu', Nu, ...
   'Lambda', Lambda, ...
   'Psi', Psi, ...
   'LB', [], ...
   'UB', [], ...
   'quadprogOptions', quadprog_options ...
);

DMCObject = initializeDMC( DMCObject );
u = u0;

for i = 1 : simulationLength

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   if i > simulation_step*(finish_time/100)                            %
      while letters_written > 0                                        %
         fprintf('\b');                                                %
         letters_written = letters_written - 1;                        %
      end                                                              %
      letters_written = fprintf('Progress:    %d%%',simulation_step);  %
      simulation_step = simulation_step + 1;                           %
   end                                                                 %
   % ================================================================= %

   t = [ time(i); time(i) + Tp ];
   z = disturbanceTrajectory(i);
   
   [ tout, xout ] = ode45( @plantFunction, t, x0, opts, u, z);

   x0 = xout( size( xout, 1 ), : );
   y = x0(4) / x0(3);

   [ DMCObject, u ] = evaluateDMC( DMCObject, ...
                                   outputDesiredTrajectory(i), ...yDesired
                                   y ); %yCurrent )

   plotControl(i) = u;
   plotOutput(i) = y;
end

fprintf('\n\n');

figure(1);
stairs(outputDesiredTrajectory,'--k');
hold on;
plot(plotOutput, 'b');
grid on;

figure(2);
hold on;
stairs(plotControl, 'r');
grid on;

%% Pierwsza kolumna komórki macierzy zawiera wielkość lambdy, 
%% druga zawiera wektor wyjścia obiektu, trzecia sterowanie.
%filename = num2str( outputDesiredTrajectory( length( outputDesiredTrajectory ) ) / 1000 );
%filename = strcat( filename, 'k_lambda_adjusting.mat' );
%load( filename );
%rows = size( results, 1 );
%results{ rows + 1, 1} = lambda;
%results{ rows + 1, 2} = plotOutput;
%results{ rows + 1, 3} = plotControl;
%save( filename, 'results' );

rmpath('../CustomToolbox/DMC');
rmpath('../PlantData/');
rmpath('../Models');
