% Skrypt realizujący sterowanie reaktorem polimeryzacji za pomocą algorytmu
% regulacji predykcyjnej DMC. W Wersji FDMC wykorzystywany jest model 
% rozmyty obiektu bazujący na trzech liniowych modelach lokalnych,
% a algorytm zaimplementowany został w wersji numerycznej.

addpath('../CustomToolboxes/fuzzy');
addpath('../Models');
addpath('../PlantData');

% Pobierany jest stan początkowy obiektu z artykułu, dla dopuszczalnego
% początku symulacji.
load('reactorData.mat');
load('inputSteadyStates.mat');

% Wektor czasu dla wstępnych symulacji obiektu służących do pozyskania
% odpowiedzi skokowych w określonych punktach pracy.
Tp = 1/60;
time = [ 0 : Tp : 10 ];

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć wartość
% tolerancji bezwzględnej i względnej metody ode45, aby na wyjściu obiektu
% ustalał się stan (a nie były widoczne oscylacje w stanie ustalonym.
opts = odeset( 'AbsTol', 1e-10, 'RelTol', 1e-12 );

stepSize = 0.0001;
dynamicsHorizon = 80;

localModelsOutputValues = [ 10000; 25000; 40000 ];

fuzzyModel = struct( ...
   'inputsNo', 1, ...
   'outputsNo', 1, ...
   'rulesNo', length(localModelsOutputValues), ...
   'uMin', 0.001, ...
   'uMax', 0.2 );
 
fuzzyModel.S = cell(0);
fuzzyModel.u0 = cell(0);
fuzzyModel.y0 = cell(0);
fuzzyModel.x0 = cell(0);
fuzzyModel.MFType = cell(0);
fuzzyModel.MFParams = cell(0);

localModelsInputs = cell( length(localModelsOutputValues), 1 );
localModelsStates = cell( length(localModelsOutputValues), 1 );
localModelsDisturbances = cell( length(localModelsOutputValues), 1 );
localModelsOutputs = cell( length(localModelsOutputValues), 1 );
localModelsStepResponses = cell( length(localModelsOutputValues), 1 );

for i = 1 : length(localModelsOutputValues)
   localModelsDisturbances{i} = 0;

   differences = abs( outputSS - localModelsOutputValues(i) );
   [ minimal, bestFittedIdx ] = min( differences );
   localModelsInputs{i} = inputSS(bestFittedIdx);

   [ tout, xout ] = ode45( @plantFunction, time, x0, opts, ...
                           localModelsInputs{i}, ...
                           localModelsDisturbances{i} );

   localModelsStates{i} = ( xout( length(xout), : ) )';
   localModelsOutputs{i} = localModelsStates{i}(4) / localModelsStates{i}(3);

   [ tout, xout ] = ode45( @plantFunction, time, localModelsStates{i}, opts, ...
                           localModelsInputs{i} + stepSize, ...
                           localModelsDisturbances{i} );

   localModelsStepResponses{i} = ( xout(:,4) ./ xout(:,3) );
   localModelsStepResponses{i} = localModelsStepResponses{i}(2:dynamicsHorizon+1);

   localModelsStepResponses{i} = ...
      ( localModelsStepResponses{i} - localModelsOutputs{i} ) / stepSize;
end

for i = 1 : length(localModelsOutputValues)
   fuzzyModel.S{i} = localModelsStepResponses{i};
   fuzzyModel.u0{i} = localModelsInputs{i};
   fuzzyModel.x0{i} = localModelsStates{i};
   fuzzyModel.y0{i} = localModelsOutputs{i};

   if i == 1
      fuzzyModel.MFType{i} = 'trapmf';
      fuzzyModel.MFParams{i} = ...
         [ - Inf; - Inf; localModelsOutputs{1}; localModelsOutputs{2} ];
   elseif i == length(localModelsOutputValues)
      fuzzyModel.MFType{i} = 'trapmf';
      fuzzyModel.MFParams{i} = ...
         [ localModelsOutputs{i-1}; localModelsOutputs{i}; Inf; Inf ];
   else
      fuzzyModel.MFType{i} = 'trimf';
      fuzzyModel.MFParams{i} = ...
         [ localModelsOutputs{i-1}; localModelsOutputs{i}; localModelsOutputs{i+1} ];
   end
end

save( '../Models/fuzzyModelTrapezoidTriangle.mat', 'fuzzyModel' );

rmpath('../CustomToolboxes/fuzzy');
rmpath('../Models');
rmpath('../PlantData');
