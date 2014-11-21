function [ MFuns ] = extractMFuns( fuzzyModel, args )

   pointsNo = length(args);
   MFuns = zeros( pointsNo, fuzzyModel.rulesNo );

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulation_step = 0;                                                %
   letters_written = 0;                                                %
   finish_time = pointsNo;                                             %
   fprintf('\n');                                                      %
   % ================================================================= %

   % Przetwarzanie wszystkich argumentów.
   for i = 1 : pointsNo
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

      % Przetwarzanie wszystkich modeli lokalnych.
      for j = 1 : fuzzyModel.rulesNo
         if( strcmp( fuzzyModel.MFType{j}, 'trimf' ) == 1)
            MFuns(i,j) = evaluateTriangleMF( args(i), fuzzyModel.MFParams{j} );
         elseif( strcmp( fuzzyModel.MFType{j}, 'trapmf' ) == 1)
            MFuns(i,j) = evaluateTrapezoidMF( args(i), fuzzyModel.MFParams{j} );
         elseif( strcmp( fuzzyModel.MFType{j}, 'gbellmf' ) == 1)
            MFuns(i,j) = evaluateGBellMF( args(i), fuzzyModel.MFParams{j} );
         end
      end
   end

   fprintf('\n\n');
end
