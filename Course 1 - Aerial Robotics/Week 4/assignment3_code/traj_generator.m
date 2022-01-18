function [ desired_state ] = traj_generator(t, ~, waypoints)

persistent waypoints0 traj_time d0 coff
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    coff = getCoff(waypoints);

else
    if t > traj_time(end)
        t = traj_time(end)-0.01;
    end
    t_index = find(traj_time >= t,1)-1;

    %     if t_index > 1
    %         t = t - traj_time(t_index-1);
    %     end

    t_index=max(t_index,1);
    if t == 0
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        scale = (t-traj_time(t_index))/d0(t_index);

        t0 = polyT(8,0,scale)';
        t1 = polyT(8,1,scale)';
        t2 = polyT(8,2,scale)';
        index = [(t_index-1)*8+1:t_index*8];
        desired_state.pos = coff(index,:)'*t0;
        desired_state.vel = coff(index,:)'*t1.*(1/d0(t_index));
        desired_state.acc = coff(index,:)'*t2.*(1/d0(t_index)^2);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
end

    function [T] = polyT(n,k,t)
        T = zeros(n,1);
        D = zeros(n,1);
        %Init:
        for i=1:n
            D(i) = i-1;
            T(i) = 1;
        end
        %Derivative:
        for j=1:k
            for i=1:n
                T(i) = T(i) * D(i);

                if D(i) > 0
                    D(i) = D(i) - 1;
                end
            end
        end

        for i=1:n
            T(i) = T(i) * t^D(i);
        end
        T = T';
    end



    function [coff, A, b] = getCoff(waypoints)
        n = size(waypoints,2)-1; % number of segments P1..n
        A = zeros(8*n, 8*n);
        b = zeros(3,8*n);

    for i=1:n
        for k=1:3
            b(k,i)=waypoints(k,i);
            b(k,i+n)=waypoints(k,i+1);
        end
    end

        row = 1;
        for i=1:n
            A(row,8*(i-1)+1:8*i) = polyT(8,0,0);
            row = row + 1;
        end

        for i=1:n
            A(row,8*(i-1)+1:8*i)=polyT(8,0,1);
            row=row+1;
        end

        for i=1:3
            A(row,1:8)=polyT(8,i,0);
            row=row+1;
        end

        for i=1:3
            A(row,end-7:end)=polyT(8,i,1);
            row=row+1;
        end

%         for i=1:6
%             A(row,1:16)=[polyT(8,i,1) -polyT(8,i,0)];
%             row=row+1;
%             A(row,9:24)=[polyT(8,i,1) -polyT(8,i,0)];
%             row=row+1;
%             A(row,17:32)=[polyT(8,i,1) -polyT(8,i,0)];
%             row=row+1;
%         end
        
        for i=2:n
            for k=1:6
                A(row,((i-2)*8)+1:i*8)= [polyT(8,k,1) -polyT(8,k,0)];
                row=row+1; 
            end
        end

        coff = inv(A)*b';
    end

end