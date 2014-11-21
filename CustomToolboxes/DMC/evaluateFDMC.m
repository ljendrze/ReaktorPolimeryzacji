function [ FDMCObject, control ] = evaluateFDMC( FDMCObject, yDesired, yCurrent )
% Komentarz odnośnie działania.

   % Wyznaczanie trajektorii zadanej ( przedłużenie wartości zadanej na cały
   % horyzont predykcji N.
   yDesiredTrajectory = zeros( FDMCObject.model.outputsNo*FDMCObject.N, 1 );
   for i = 1 : FDMCObject.N
      for j = 1 : FDMCObject.model.outputsNo
         yDesiredTrajectory( (i-1)*FDMCObject.model.outputsNo + j) = yDesired(j);
      end
   end

   % Wyznaczenie sił odpalenia reguł na podstawie ostatniej wartości wyjścia obiektu.
   weights = obtainFiringStrengths( FDMCObject.model, yCurrent );

   deltaU = zeros( FDMCObject.Nu * FDMCObject.model.inputsNo, 1 );
   for i = 1 : FDMCObject.model.rulesNo
      deltaU = deltaU + weights(i)*FDMCObject.K{i}*( yDesiredTrajectory - ...
         ( yCurrent + FDMCObject.Mp{i}*FDMCObject.deltaUPast ) );
   end

   % Wyznaczanie nowej wartości sterowania i rzutowanie na zbiór ograniczeń.
   if( FDMCObject.uPast( 1 : FDMCObject.model.inputsNo ) + ...
   deltaU( 1 : FDMCObject.model.inputsNo ) > FDMCObject.model.uMax )
      deltaU( 1 : FDMCObject.model.inputsNo ) = ...
         FDMCObject.model.uMax - FDMCObject.uPast( 1 : FDMCObject.model.inputsNo );
      control = FDMCObject.model.uMax;
   elseif( FDMCObject.uPast( 1 : FDMCObject.model.inputsNo ) + ...
   deltaU( 1 : FDMCObject.model.inputsNo ) < FDMCObject.model.uMin )
      deltaU( 1 : FDMCObject.model.inputsNo ) = ...
         FDMCObject.model.uMin - FDMCObject.uPast( 1 : FDMCObject.model.inputsNo );
      control = FDMCObject.model.uMin;
   else
      control = FDMCObject.uPast( 1 : FDMCObject.model.inputsNo ) + ...
         deltaU( 1 : FDMCObject.model.inputsNo );
   end

   plot( ( yCurrent + FDMCObject.Mp{2}*FDMCObject.deltaUPast ) );

        

   % Aktualizacja wektorów przeszłych sterowań i przyrostów sterowań.
   FDMCObject.deltaUPast = [ deltaU; ...
      FDMCObject.deltaUPast( 1 : (FDMCObject.D-2)*FDMCObject.model.inputsNo ) ];
   FDMCObject.uPast = [ control; ...
      FDMCObject.deltaUPast( 1 : (FDMCObject.D-2)*FDMCObject.model.inputsNo ) ];




end
