function [ DMC, control ] = evaluateDMC( DMC, yDesired, yCurrent )
% Komentarz odnośnie działania.

   % Obiekt jest wymiaru m x n   

   yDesiredTrajectory = zeros( DMC.model.outputNo*DMC.N, 1 );
   for i = 1 : DMC.N
      for j = 1 : DMC.model.outputNo
         yDesiredTrajectory( (i-1)*DMC.model.outputNo + j) = yDesired(j);
      end
   end

   DMC.yFree = yCurrent*ones( DMC.model.outputNo*DMC.N, 1 ) + ...
      DMC.Mp*DMC.deltaUPast;

   % TODO
   % Wektor b do sprawdzenia. 
   for i = 1 : DMC.model.inputNo * DMC.Nu
      DMC.b(i) = ...
         DMC.uPast( 1 : DMC.model.inputNo ) - ...DMC.model.u0 - ...
         DMC.model.uMin;
   end
   for i = DMC.model.inputNo*DMC.Nu+1 : 2*DMC.model.inputNo*DMC.Nu
      DMC.b(i) = ...
         - DMC.uPast( 1 : DMC.model.inputNo ) + ...DMC.model.u0 + ...
         DMC.model.uMax;
   end

   DMC.f = -2*DMC.M' * DMC.Psi*( ...
      yDesiredTrajectory - DMC.yFree );

   %display( DMC.yFree );
   %display( DMC.b );
   %pause;

   deltaU = quadprog( DMC.H, DMC.f, ...
                      DMC.A, DMC.b, ...
                      DMC.Aeq, DMC.beq, ...
                      DMC.LB, DMC.UB, ...
                      DMC.quadprogX0, DMC.quadprogOptions );

   % display(deltaU);

   control = DMC.uPast( 1 : DMC.model.inputNo ) + ...
      deltaU( 1 : DMC.model.inputNo );

   DMC.deltaUPast = [ deltaU; ...
      DMC.deltaUPast( 1 : (DMC.D-2)*DMC.model.inputNo ) ];
   DMC.uPast = [ control; ...
      DMC.deltaUPast( 1 : (DMC.D-2)*DMC.model.inputNo ) ];

end
