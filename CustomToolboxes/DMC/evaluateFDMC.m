function [ FDMCObject, control ] = evaluateFDMC( FDMCObject, yDesired, yCurrent )
% Komentarz odnośnie działania.

   % Wyznaczanie trajektorii zadanej ( przedłużenie wartości zadanej na cały
   % horyzont predykcji N.
   yDesiredTrajectory = zeros( FDMCObject.model.outputSize*FDMCObject.N, 1 );
   for i = 1 : FDMCObject.N
      for j = 1 : FDMCObject.model.outputSize
         yDesiredTrajectory( (i-1)*FDMCObject.model.outputSize + j) = yDesired(j);
      end
   end

   % Wyznaczenie sił odpalenia reguł na podstawie ostatniej wartości wyjścia obiektu.
   weights = obtainFiringStrengths( FDMCObject.model, yCurrent );

   deltaU = zeros( FDMCObject.Nu * FDMCObject.model.inputSize, 1 );
   for i = 1 : FDMCObject.model.rulesNo
      deltaU = deltaU + weights(i)*FDMCObject.K{i}*( yDesiredTrajectory - ...
         ( yCurrent + FDMCObject.Mp{i}*FDMCObject.deltaUPast ) );
   end

   % Wyznaczanie nowej wartości sterowania i rzutowanie na zbiór ograniczeń.
   if( FDMCObject.uPast( 1 : FDMCObject.model.inputSize ) + ...
   deltaU( 1 : FDMCObject.model.inputSize ) > FDMCObject.model.uMax )
      deltaU( 1 : FDMCObject.model.inputSize ) = ...
         FDMCObject.model.uMax - FDMCObject.uPast( 1 : FDMCObject.model.inputSize );
      control = FDMCObject.model.uMax;
   elseif( FDMCObject.uPast( 1 : FDMCObject.model.inputSize ) + ...
   deltaU( 1 : FDMCObject.model.inputSize ) < FDMCObject.model.uMin )
      deltaU( 1 : FDMCObject.model.inputSize ) = ...
         FDMCObject.model.uMin - FDMCObject.uPast( 1 : FDMCObject.model.inputSize );
      control = FDMCObject.model.uMin;
   else
      control = FDMCObject.uPast( 1 : FDMCObject.model.inputSize ) + ...
         deltaU( 1 : FDMCObject.model.inputSize );
   end

   plot( ( yCurrent + FDMCObject.Mp{2}*FDMCObject.deltaUPast ) );

        

   % Aktualizacja wektorów przeszłych sterowań i przyrostów sterowań.
   FDMCObject.deltaUPast = [ deltaU; ...
      FDMCObject.deltaUPast( 1 : (FDMCObject.D-2)*FDMCObject.model.inputSize ) ];
   FDMCObject.uPast = [ control; ...
      FDMCObject.deltaUPast( 1 : (FDMCObject.D-2)*FDMCObject.model.inputSize ) ];




end
