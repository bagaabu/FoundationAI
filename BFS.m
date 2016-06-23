%% set the variables
clear all;
% MapInit=[1 0 0 0;0 0 0 0;0 0 2 0;0 4 3 0];
% MapInit=[0 0 0 0;0 1 0 0;0 2 0 4;0 3 0 0];
% MapInit=[0 0 0 0;0 1 0 0;0 2 0 0;0 4 3 0];
% MapInit=[1 0 0 0;0 0 0 0;4 0 0 2;0 0 0 3];
MapInit=[4 0 2 3;0 0 0 0;0 0 0 0;0 0 0 1];
goal=[0 0 0 0;0 1 0 0;0 2 0 0;0 3 0 4];
Map=MapInit;
Maptemp=Map;
depth=zeros(100,1);
breth=zeros(100,1);
stack=cell(1000000,1);
route=cell(50,1);
route_index=zeros(50,1);
stack{1}=MapInit;
path=zeros(1000000,4);
direction=[-1,0;1,0;0,-1;0,1];
bip=[4,4];%blockinitposition
temp=bip;
score=0;
j=1;
m=0;
%% Main code
 while score==0;
    m=m+1;
    Map=stack{m};
    Maptemp=Map;
%% Random create 4 Directions
    direction_index=randperm(4); % creat a list of 4 random number
    for l=1:4 
    b(l,:)=direction(direction_index(:,l),:);
    end
    [row,col]=find(Map==4);
    bip=[row,col];
    temp=bip;
    start=j; % used to judge how many states expanded
%%  Expand the Node
    for n=1:4
        bip=bip+b(n,:); % move the block
        moverror=find(bip<1|bip>4); % is the move out of the board
        if (~isempty(moverror))
            bip=temp;
        elseif b(n,:)==path(m,1:2) % is the move goback to the previous state
            bip=temp;
        else %creat the new state
            Maptemp(bip(1,1),bip(1,2))=Map(temp(1,1),temp(1,2));
            Maptemp(temp(1,1),temp(1,2))=Map(bip(1,1),bip(1,2));
            for x=1:j % is this state has been expanded before
                if stack{x}==Maptemp
                    bip=temp;
                    Maptemp=Map;
                    break
                else % put the new state to the stack,wait for the next move
                    j=j+1;
                    stack{j}=Maptemp;
                    path(j,1:2)=-b(n,:);% record which direction it cames
                    path(j,3)=path(m,3)+1;%record the depth
                    path(j,4)=m;%record the position of it's father
                    bip=temp;
                    Maptemp=Map;
                end
            end
        end
    end
%% Compare have it meet the goal
    stop=j;
    for k=start:stop % compare all expanded nodes
        if goal==stack{k};
            steps=path(j,3);
            score=1;
            routemp=k;
            for r=1:steps % track the route
                route_index(r,:)=path(routemp,4);
                routemp=path(routemp,4);
                route{r}=stack{route_index(r,:)};
            end
            return
        else score=0;    
        end
    end
 end

 