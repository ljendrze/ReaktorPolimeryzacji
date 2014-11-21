fileList = cell(4,1);

fileList{1} = '10k_horizon_adjusting.mat';
fileList{2} = '20k_horizon_adjusting.mat';
fileList{3} = '30k_horizon_adjusting.mat';
fileList{4} = '40k_horizon_adjusting.mat';

precisionLB = [ 0.99*10000, 0.99* 20000, 0.99*30000, 0.99*40000 ];
precisionUB = [ 1.01*10000, 1.01* 20000, 1.01*30000, 1.01*40000 ];

for k = 1 : length( fileList )
   load( fileList{k} );
   % figure(k);
   hold on;   
   predictionHorizons = [ 80, 70, 60, 50, 40, 30, 20, 10 ];
   steeringHorizons = [ 30, 20, 18, 16, 14, 12, 10, 8, 6, 5, 4, 3, 2, 1 ];

   n = length( predictionHorizons );
   colors = distinguishable_colors( n );
   
   figure( (k-1)*2 + 1 );
   hold on;
   labels = cell( n,1 );
   for i = 1 : n
      temp = results{predictionHorizons(i), predictionHorizons(i)};
      plot( temp(:,1), 'Color', colors(i,:) );
      labels{i} = num2str( predictionHorizons(i) );
   end
   grid on;
   legend( labels );
   plot( [1, size(temp,1)], [precisionUB(k), precisionUB(k)], '--k');
   plot( [1, size(temp,1)], [precisionLB(k), precisionLB(k)], '--k');
   
   
   n = length( steeringHorizons );
   colors = distinguishable_colors( n );
   
   figure( k*2 );
   hold on;
   labels = cell( n,1 );
   for i = 1 : n
      temp = results{ 30, steeringHorizons(i)};
      plot( temp(:,1), 'Color', colors(i,:) );
      labels{i} = num2str( steeringHorizons(i) );
   end
   grid on;
   legend( labels );
   plot( [1, size(temp,1)], [precisionUB(k), precisionUB(k)], '--k');
   plot( [1, size(temp,1)], [precisionLB(k), precisionLB(k)], '--k');
end
