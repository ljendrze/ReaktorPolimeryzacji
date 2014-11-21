function [ weights, nn_weights ] = obtainFiringStrengths( fuzzyModel, input )
% Model rozmyty powinien być macierzą komórkową struktur, w której będą
% przetwarzane kolejne modele lokalne. Modele lokalne mają postać 
% znormalizowanej odpowiedzi skokowej.
%
% Argumenty wejściowe funkcji:
%    - deltaUPast - wektor przeszłych zmian sterowania w obiekcie
%    - y_k - wartość wyjścia obiektu w chwili k-tej.
%    - algorithm - string przyjmujący wartości 'FDMC', 'FDMC-SL',
%      'FDMC-NPL' obliczający wymagane przez dane podejście elementy.
%
% Struktura modelu lokalnego powinna zawierać pola:
%    - S - odpowiedź skokowa modelu lokalnego;
%    - M - macierz dynamiczna modelu lokalnego;
%    - Mp - macierz reprezentująca liniowy wpływ poprzednich wejść
%    - x0 - wartość wejścia modelu lokalnego w punkcie pracy;
%    - y0 - wartość wyjścia modelu lokalnego w punkcie pracy;
%    - MFType - string, typ funkcji przynależności;
%    - MFParams - parametry funkcji przynależności do obliczenia
%      siły odpalenia reguły;
%
% Osobne funkcjonalności modelu rozmytego:
%    - [ new_output] = evaluateFuzzyModel( prev_output )
%      Za pomocą tej funkcji będzie obliczana kolejna wartość wyjścia
%      modelu rozmytego. 
%       * W przypadku algorytmu FDMC obliczana będzie wartość funkcji przynależności
%         tak aby pozyskać siły odpalenia reguł. W głównej pętli wykorzystywane 
%         będą wyznaczone parametry liniowego modelu lokalnego w celu wyznaczenia 
%         jego sterowania obiektu, a wagi posłużą do ważenia odpowiadających
%         obliczonych przyrostów sterowania. 
%       * W algorytmie FDMC-SL w każdej iteracji obliczamy nową zlinearyzowaną
%         odpowiedź swobodną obiektu na podstawie raz w każdej iteracji rozmytej
%         wartości macierzy Mp.
%       * W algorytmie FDMC-NPL wykorzystywane będą bezpośrednio elementy
%         odpowiedzi swobodnej obiektu. W każdej iteracji, podczas wyznaczania
%         każdego z elementów trajektorii swobodnej rozmywane będą elementy wektora
%         S, a następnie odpowiednio przesunięty wektor deltaUPast będzie mnożony
%         z lewej strony przez odpowiadające współczynniki znormalizowanej odpowiedzi
%         skokowej i dodawana będzie różnica pomiędzy wartością pomiaru 
%         a wyznaczoną przez model = d_k.
%    - [ weights, ]

% Wektor S wygodniej będzie zrobić na komórkach - łatwiej będzie dla MIMO.

   weights = zeros( fuzzyModel.rulesNo, 1 );
   
   % Przetwarzanie wszystkich modeli lokalnych
   for i = 1 : fuzzyModel.rulesNo
      if( strcmp( fuzzyModel.MFType{i}, 'trimf' ) == 1)
         weights(i) = evaluateTriangleMF( input, fuzzyModel.MFParams{i} );
      elseif( strcmp( fuzzyModel.MFType{i}, 'trapmf' ) == 1)
         weights(i) = evaluateTrapezoidMF( input, fuzzyModel.MFParams{i} );
      elseif( strcmp( fuzzyModel.MFType{i}, 'gbellmf' ) == 1)
         weights(i) = evaluateGBellMF( input, fuzzyModel.MFParams{i} );
      end
   end
   nn_weights = weights;
   weights = weights / sum(weights);
end
