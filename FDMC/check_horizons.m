D = 80;
predictionHorizons = [ 80, 70, 60, 50, 40, 30, 20, 18, 16, 14, 12, 10, 8, 6, 4 ];
steeringHorizons = [ 80, 70, 60, 50, 40, 30, 20, 18, 16, 14, 12, 10, 8, 6, 5, 4, 3, 2, 1 ];
for ii = 1 : length(predictionHorizons)
N = predictionHorizons(ii);
for jj = ii : length(steeringHorizons)
Nu = steeringHorizons(jj);

display(N); display(Nu);

% Skrypt realizujący sterowanie reaktorem polimeryzacji za pomocą algorytmu
% regulacji predykcyjnej DMC. W wersji podstawowej wykorzystywany jest
% model liniowy obiektu, a algorytm zaimplementowany został w wersji
% numerycznej.

% Wartości parametrów określąjace punkt równowagi reaktora polimeryzacji 
% opisywanego w artykule.
x0 = [5.50677 0.132906 0.0019752 49.3818];
y0 = 25000.5;
u0 = 0.016783;
z0 = 0;

% Czas dla wstępnej symulacji reaktora polimeryzacji.
time = [0:1/60:10];

% Wartość skoku wykorzystana do otrzymania modelu w postaci odpowiedzi
% na skok jednostkowy (otrzymana odpowiedź zostanie znormalizowana).
du = 0.001;
u = u0 + du;

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć wartość
% tolerancji bezwzględnej i względnej metody ode45, aby na wyjściu obiektu
% ustalał się stan (a nie były widoczne oscylacje w stanie ustalonym).
opts=odeset('AbsTol', 1e-6, 'RelTol', 1e-8);

% Symulacja obiektu wykorzystująca funkcję różniczkującą ode45 posługującą
% się metodą Rungego-Kutty 4. rzedu.
[ tout, yout ] = ode45(@sfun, time, x0, opts, u, z0);
plantOutput = yout(:,4) ./ yout(:,3);

% Horyzonty dynamiki, predykcji i sterowania. Na początku strojenia zostają
% wybrane jako D = N = 2*Nu.
%D = 80;
%N = 80;
%Nu = 40;
%D = 80;
%N = 10;
%Nu = 2;

% Określenie odpowiedzi skokowej - brane jest tylko D elementów
S = plantOutput( 1 : D );

% Krok normalizacyjny - odpowiedź skokowa wykorzystywana w algorytmie DMC
% musi zostać znormalizowana!
S = ( S - y0 ) / du;

% Przygotowanie wymaganych macierzy Psi, Lambda, Mp i M:
lambda = 1e13;
Lambda = lambda*eye(Nu);

psi = 1;
Psi = psi*eye(N);


Mp = zeros(N,D-1); % Rozmiar N x (D-1)
Mp = compute_Mp( S, N, D );

M = zeros(N,Nu); % Rozmiar N x Nu
M = compute_M( S, N, Nu );

% Obliczenie off-line wzmocnień.
K = (M' * Psi * M + Lambda)^(-1)*M'*Psi;

% =============== ANALIZA WARTOŚCI PARAMETRÓW NA DIAG(M' * M) =============== %
% Aby ustalić dobrą wartość parametru lambda, dobrze jest porownać wartości   %
% znajdujące się na diagonali macierzy [M' * M], ponieważ do obliczania       %
% wzmocnień w algorytmie DMC do elementów na przekątnej tej macierzy          %
% dodajemy macierz diagonalną Lambda (Lambda = lambda*eye).                   %
% 
% diag_M = zeros(1,Nu);
% M_form_product = M' * M;
% for i = 1 : Nu
%    diag_M(i) = M_form_product(i,i);
% end
% 
% figure(3);
% Handle = axes;
% set(Handle,'FontName','fixed');
% hold on;
% plot([1:1:size(diag_M, 2)], diag_M,'o');
% =========================================================================== %

% Skracanie czasu symulacji, dla przejrzystszych wykresów.
time = [0:1/60:5];

simulationLength = size(time,2);

% Stworzenie i obliczenie wektora wartości zadanej.
outputDesiredTrajectory = y0 * ones(1, simulationLength);

step_time = 10;
for i = step_time : simulationLength
   outputDesiredTrajectory(i) = 30000;
end

disturbanceTrajectory = zeros(simulationLength,1);
for i = 30 : simulationLength
   disturbanceTrajectory(i) = 0;
end


% Przygotowanie wektorów sterowania i wartości. Wektory te będą
% wykorzystane do rysowania wykresów schodkowych.
plotOutput = y0 * ones( simulationLength, 1 );

u = u0;
plotControl = u0*ones( simulationLength, 1 );
 
% Przygotowanie wektora zmian wartości sterowania.
deltaUPast = zeros(D-1,1);

Tp = 1/60;
sim_x0 = x0;

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulation_step = 0;                                                %
letters_written = 0;                                                %
finish_time = simulationLength;                                           %
fprintf('\n');                                                      %
% ================================================================= %

controlIncrement = zeros(Nu, 1);

y_desired = zeros(N, 1);

J = zeros(Nu);
for i = 1 : Nu
   for j = 1 : i
      J(i,j) = 1;
   end
end

A = [ -J; J ];

b = zeros(2*Nu,1);

u_min = 0.001;
u_max = 0.2;
quadprog_options = optimset('Algorithm','active-set','Display','off');
Aeq = [];
beq = [];
LB = ones(Nu,1);
LB = -Inf * LB;
UB = ones(Nu,1);
UB = Inf * UB;
quadprog_x0 = zeros( Nu, 1 );

% H może zostać wyrzucone poza pętlę, ponieważ model się nie zmienia, tzn.
% zarówno macierz M jak i Lambda nie zmieniają wartości swoich elementów.
H = 2*(M'*Psi*M + Lambda);

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

   if ( i == simulationLength)
      sim_time = [ 10, 10 + 1/60 ];
   else
      sim_time = [time(i) time(i+1)];
   end
   
   [sim_t, sim_x] = ode45(@sfun, sim_time, sim_x0, opts, u, disturbanceTrajectory(i));

   sim_x0 = sim_x(size(sim_x,1),:);
   sim_y = sim_x0(4) / sim_x0(3);

   y_desired = outputDesiredTrajectory(i)*ones(N,1);
   y_free = sim_y*ones(N, 1) + Mp*deltaUPast;
   f = -2*M'*Psi*( y_desired - y_free );
   
   for j = 1 : Nu
      b(j) = -u_min + u;
   end
   for j = Nu+1 : 2*Nu
      b(j) = u_max - u;
   end

   controlIncrement = quadprog( H, f,...
                       A, b,...
                       Aeq, beq,...
                       LB, UB,...
                       quadprog_x0, quadprog_options );

   if i-1 > 0
      plotControl(i) = plotControl(i-1) + controlIncrement(1);
   else
      plotControl(i) = u + controlIncrement(1);
   end
   
   for j=D-1 : -1 : 2
      deltaUPast(j) = deltaUPast(j-1);
   end

   plotOutput(i) = sim_y;
   u = plotControl(i);
   deltaUPast(1) = controlIncrement(1);
end

fprintf('\n\n');

% overshoot = find_overshoot( plotOutput, outputDesiredTrajectory(step_time), step_time );

% plot_time = out_t(1:size(plotOutput, 2));
% plot_arg = [1:size(plotOutput, 2)];
% figure(1);
% stairs(time,outputDesiredTrajectory,'--k');
% hold on;
% plot(time, plotOutput, 'b');
% grid on;
% figure(2);
% stairs(time, plotControl, 'r');
% grid on;

% figure(1);
% figure(1);
% stairs(outputDesiredTrajectory,'--k');
% hold on;
% plot(plotOutput, 'b');
% grid on;
% 
% figure(2);
% hold on;
% stairs(plotControl, 'b');
% grid on;

load( '30k_horizon_adjusting.mat' );
results{N,Nu} = [plotOutput, plotControl];
save( '30k_horizon_adjusting.mat', 'results' );

end
end

load( '30k_horizon_adjusting.mat' );
save( '30k_horizon_adjusting.mat', 'results', 'predictionHorizons', 'steeringHorizons' );
