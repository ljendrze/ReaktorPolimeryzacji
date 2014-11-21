% Skrypt realizujący sterowanie reaktorem polimeryzacji za pomocą algorytmu
% regulacji predykcyjnej DMC. W Wersji FDMC wykorzystywany jest model 
% rozmyty obiektu bazujący na trzech liniowych modelach lokalnych,
% a algorytm zaimplementowany został w wersji numerycznej.

addpath('../CustomToolboxes/DMC');
addpath('../CustomToolboxes/fuzzy');
addpath('../Models');
addpath('../PlantData');

load( 'fuzzyModelTrapezoidTriangle.mat' );
load( '../PlantData/reactorData.mat' );

% Horyzonty dynamiki, predykcji i sterowania.
D = 80;
N = 30;
Nu = 1;

Tp = 1/60;

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć wartość
% tolerancji bezwzględnej i względnej metody ode45, aby na wyjściu obiektu
% ustalał się stan (a nie były widoczne oscylacje w stanie ustalonym.
opts = odeset( 'AbsTol', 1e-6, 'RelTol', 1e-8 );

% Przygotowanie wymaganych macierzy Psi i Lambda.
lambda = 1e11;
Lambda = lambda * eye( Nu );

psi = 1;
Psi = psi * eye( N );


% Skracanie czasu symulacji, dla przejrzystszych wykresów.
time = [ 0 : Tp : 2 ];

simulationLength = length( time );

% Stworzenie i obliczenie wektora wartosci zadanej.
outputDesiredTrajectory = y0 * ones( simulationLength, 1 );

for i = 10 : simulationLength
   outputDesiredTrajectory(i) = 30000;
end

% Trajektoria zmian zakłóceń.
disturbanceTrajectory = z0 * ones( simulationLength, 1 );

for i = 10 : simulationLength
   disturbanceTrajectory(i) = 0;
end

% Przygotowanie wektorow sterowania i wartosci. Wektory te beda
% wykorzystane do rysowania wykresow schodkowych.
plotOutput = y0 * ones( simulationLength, 1 );
plotControl = u0 * ones( simulationLength, 1 );

% Wartość sygnału sterującego.
u =  u0;
 
% Wektor zmiennych stanu i bieżąca wartość wyjścia obiektu.
plantState = x0;
plantOutput = y0;

% Wektor odpowiedzi swobodnej modelu.
modelFreeResponses = cell( length( fuzzyModel ), 1 );

% Wektor przeszłych przyrostów sterowania.
deltaUPast = zeros( D-1, 1 );

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulation_step = 0;                                                %
letters_written = 0;                                                %
finish_time = simulationLength;                                     %
fprintf('\n');                                                      %
% ================================================================= %

FDMCObject = struct( ...
   'model', fuzzyModel, ...
   'uPast', u0*ones( fuzzyModel.inputsNo*D, 1 ), ...
   'deltaUPast', zeros( fuzzyModel.inputsNo*(D-1), 1 ), ...
   'D', 80, ...
   'N', 30, ...
   'Nu', 1, ...
   'Lambda', Lambda, ...
   'Psi', Psi ...
);

FDMCObject = initializeFDMC( FDMCObject );






control = u0;

% Właściwa pętla algorytmu FDMC.
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

   disturbance = disturbanceTrajectory(i);

   simulationTime = [ time(i), time(i)+Tp ];
   
   [ tout, xout ] = ode45( @plantFunction, simulationTime, plantState, ...
                           opts, control, disturbance );

   plantState = xout( size( xout, 1 ), : );
   plantOutput = plantState(4) / plantState(3);

   [ FDMCObject, control ] = evaluateFDMC( ...
      FDMCObject, outputDesiredTrajectory(i), plantOutput );
   
   plotOutput(i) = plantOutput;
   plotControl(i) = control;
   
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


rmpath('../CustomToolboxes/DMC');
rmpath('../CustomToolboxes/fuzzy');
rmpath('../Models');
rmpath('../PlantData');
