classdef LinSys < handle
   properties (SetAccess=protected)
       Hu;
       hu;   %Input constraints
       Hd;
       hd;   %Disturbance constraints
       A;
       B;
       E;
       K;     %System dynamics
   end
   methods
       function s = LinSys(Hu, hu, A, B, E, K)
           s.Hu = Hu;
           s.hu = hu;
           s.A = A;
           s.B = B;
           s.E = E;
           s.K = K;
       end
       function setd(s, Hd, hd)
           s.Hd = Hd;
           s.hd = hd;
       end
       function x_0 = Pre(s,C)
           %define matrices s.t. the A, B ineq. matrices for polyhedron s
           % are A = [Hxu Hdd] and B = h
           
           Hxu = [C.A*s.A C.A*s.B; ...
               zeros(size(s.Hu,1),size(C.A*s.A,2)) s.Hu; ...
               zeros(size(s.Hd,1),size([C.A*s.A C.A*s.B],2))];
           Hdd = [C.A*s.E; zeros(size(s.hu,1),size(s.Hd,2)); s.Hd];
           h = [C.b - C.A*s.K; s.hu; s.hd];
           D = Polyhedron(s.Hd, s.hd);      %define Polyhedron for D
           v_i = D.V;                       %find vertices v_i of D

           if(size(v_i, 1) ~= 0)
               int = Polyhedron(Hxu, ...    %define intersection Polyhedron
                   h - Hdd*v_i(1));
           else
               int = Polyhedron(Hxu, h);
           end
           if(size(v_i, 1) > 1)        %intersect through all vertices of D
               for i = 2:size(v_i, 1)
                   int = intersect(int, Polyhedron(Hxu, ...
                       h - Hdd*v_i(i)));
               end
           end
           
           x_0 = projection(int, 1:size(s.A,2));
       end
       function [C, iter] = ConInvOI(s,G)
           iter = 0;
           C = G;
           C_plus = intersect(G,Pre(s,C));
           while ~isEmptySet(C\C_plus)
               iter = iter + 1;
               disp(['Iteration: ' num2str(iter)]);
               C = C_plus;
               C_plus = intersect(G,Pre(s,C));
           end
       end
   end
end