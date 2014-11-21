% Wartości parametrów okresląjace punkt równowagi reaktora polimeryzacji 
% opisywanego w artykule. Zmienne stanu, wartość sterowania i zakłócenie
% w postaci różnicy w stosunku do strumienia dopływającego.

addpath('../PlantData');
load( 'reactorData.mat' );

% Czas symulacji obiektu.
time = [ 0 2 ]';

% Ze względu na postać różniczkowej funkcji stanu, należy zwiększyć 
% wartość tolerancji bezwzględnej i względnej metody ode45, 
% aby na wyjściu obiektu ustalał się stan (a nie były widoczne 
% oscylacje w stanie ustalonym).
opts = odeset('AbsTol',1e-8,'RelTol',1e-10);

[ tout, xout ] = ode45( @plantFunction, time, x0, opts, 0.001, z0);
y = xout( :, 4 ) ./ xout( :, 3 );

plot( tout, y );
grid on;
ylabel('Wyjscie obiektu');
xlabel('Czas');

rmpath('../PlantData');
