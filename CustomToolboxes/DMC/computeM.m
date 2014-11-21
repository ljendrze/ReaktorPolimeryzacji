function [ M ] = computeM( inputNo, outputNo, S, N, Nu )
% Oblicza macierz dynamiczną M wykorzystywaną w algorytmie DMC. Macierz M
% jest rozmiaru N x Nu.
% 
% Argumenty funkcji:
% 
%    - S  - wektor odpowiedzi skokowej
%    - N  - horyzont predykcji algorytmu
%    - Nu - horyzont sterowania algorytmu
% 
% Wartości zwracane:
% 
%    - M - macierz dynamiczna obiektu

   % S - ny x nu

   M = zeros( outputNo*N, inputNo*Nu );

   for i = 1 : N
      for j = 1 : Nu
         % display(i); display(j);
         if i-j+1 > 0
            M( (i-1)*outputNo+1 : i*outputNo, (j-1)*inputNo+1 : j*inputNo ) = ...
               S( (i-j)*outputNo+1 : (i-j+1)*outputNo, 1 : inputNo);
         end
      end
   end

end
