function [ Mp ] = computeMp( inputNo, outputNo, S, N, D )
% Oblicza macierz Mp wykorzystywaną w algorytmie DMC do obliczania
% składowej swobodnej wyjścia obiektu. 
% Macierz M jest rozmiaru N x Nu.

   Mp = zeros( outputNo*N, (D-1)*inputNo );
   
   for i = 1 : N
      for j = 1 : D-1
         if i+j > D
            Mp( (i-1)*outputNo+1 : i*outputNo, (j-1)*inputNo+1 : j*inputNo ) = ...
               S( (D-1)*outputNo+1 : D*outputNo, : ) - ...
               S( (j-1)*outputNo+1 : j*outputNo, : );
         else
            Mp( (i-1)*outputNo+1 : i*outputNo, (j-1)*inputNo+1 : j*inputNo ) = ...
               S( (i+j-1)*outputNo+1 : (i+j)*outputNo, : ) - ...
               S( (j-1)*outputNo+1 : j*outputNo, : );
         end
      end
   end

end
