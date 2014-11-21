function dxdt = plantFunction(t, x, u, z)
% sfun(t,x,u) - funkcja obliczająca równania stanu dla 
%               układu dynamicznego reaktora polimeryzacji.
% 
%    ARGUMENTY:
%      t - czas
%      x - wartosc stanu
%      u - wartosc sterowania
%      z - parametr zaklocenia
%    WARTOSCI WYJSCIOWE:
%      dxdt - wartosc pochodnej po czasie funckji stanu

dxdt = [ ...
   10*(1 + z)*(6 - x(1)) - 2.4568*x(1)*sqrt(x(2));
   80*u - 0.1022*x(2) - 10*(1 + z)*x(2);
   0.0024121*x(1)*sqrt(x(2)) + 0.112191*x(2) - 10*(1 + z)*x(3);
   245.978*x(1)*sqrt(x(2)) - 10*(1 + z)*x(4)
];
