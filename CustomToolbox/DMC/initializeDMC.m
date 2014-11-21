function [ DMC ] = initializeDMC( DMC, yDesired, yCurrent )
% Komentarz odnośnie działania.


   % TODO
   % Dorobić dla obiektów MIMO.
   DMC.deltaUPast = zeros( DMC.model.inputNo*(DMC.D-1), 1 );
   DMC.uPast = DMC.model.u0*ones( DMC.model.inputNo * DMC.D, 1 );

   DMC.M = computeM( DMC.model.inputNo, ...
                           DMC.model.outputNo, ...
                           DMC.model.S, ...
                           DMC.N, ...
                           DMC.Nu );

   DMC.Mp = computeMp( DMC.model.inputNo, ...
                             DMC.model.outputNo, ...
                             DMC.model.S, ...
                             DMC.N, ...
                             DMC.D );

   % TODO
   % Dorobić jeszcze dla obiektów MIMO
   J = zeros( DMC.Nu);
   for i = 1 : DMC.Nu
      for j = 1 : i
         J(i,j) = 1;
      end
   end
   
   DMC.A = [ -J; J ];
   DMC.b = zeros( 2*DMC.Nu, 1 );

   DMC.H = 2*( DMC.M' * DMC.Psi * DMC.M + DMC.Lambda );
   DMC.f = zeros( DMC.model.inputNo*DMC.Nu, 1);

   % TODO
   % Sprawdzić czy te elementy w ogóle są potrzebne.
   DMC.Aeq = [];
   DMC.beq = [];

   DMC.quadprogX0 = zeros( DMC.model.inputNo*DMC.Nu, 1 );
end
