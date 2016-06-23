clear all;
tic;
% MapInit=[0 0 0 0;0 0 0 0;0 0 0 0;1 2 3 4];
% MapInit=[0 0 2 3;0 0 0 0;0 0 0 0;4 0 0 1];
% MapInit=[1 0 0 0;0 0 0 0;4 0 0 2;0 0 0 3];
MapInit=[0 0 0 0;0 1 0 0;0 0 2 0;0 4 3 0];
goal=[0 0 0 0;0 1 0 0;0 2 0 0;0 3 0 4];
Map=MapInit;
Maptemp=Map;
depth=zeros(100,1);
breth=zeros(100,1);
stack=cell(10000,1);
tempstack=cell(4,1);
openstack=cell(10000,1);
openstack{1}=MapInit;
closedstack=cell(10000,1);
closedstack{1}=MapInit;
openstack_index=1;
openpath=zeros(10000,5);
route=cell(50,1);
route_index=zeros(50,1);
stack{1}=MapInit;
score=0;
path=zeros(100000,5);
direction=[-1,0;1,0;0,-1;0,1];
bip=[4,4];%blockinitposition
temp=bip;
m=0;

   while score==0
    m=m+1;
    Map=stack{m};
    Maptemp=Map;
    direction_index=randperm(4);
    tempath=zeros(4,4);
    tempstack=cell(4,1);
    Manhatan=[];
    f=0;
    for l=1:4
    b(l,:)=direction(direction_index(:,l),:);
    end
    [row,col]=find(Map==4);
    bip=[row,col];
    temp=bip;
    start=f;
    for n=1:4
        bip=bip+b(n,:);
        moverror=find(bip<1|bip>4);
        if (~isempty(moverror))
            bip=temp;
        elseif b(n,:)==path(m,1:2)
            bip=temp;
        else
            Maptemp(bip(1,1),bip(1,2))=Map(temp(1,1),temp(1,2));
            Maptemp(temp(1,1),temp(1,2))=Map(bip(1,1),bip(1,2));
            f=f+1;
            tempstack{f}=Maptemp;
            tempath(f,1:2)=-b(n,:);
            tempath(f,3)=path(m,3)+1;
            tempath(f,4)=m;
            bip=temp;
            Maptemp=Map;        
        end
    end
    stop=f;
    %% Compute the manhattan distance
    %find the 4 characters position
    for c=1:stop-start
        [row,col]=find(tempstack{c}==1);
        signa(c,:)=[row,col];
        [row,col]=find(tempstack{c}==2);
        signb(c,:)=[row,col];
        [row,col]=find(tempstack{c}==3);
        signc(c,:)=[row,col];        
        [row,col]=find(tempstack{c}==4);
        signblock(c,:)=[row,col];
        % This is the evalutation function
        Manhatan(1,c)=abs(signa(c,1)-2)+abs(signa(c,2)-2)+abs(signb(c,1)-3)+abs(signb(c,2)-2)+abs(signc(c,1)-4)+abs(signc(c,2)-2)+abs(signblock(c,1)-4)+abs(signblock(c,2)-4);
        tempath(c,5)=Manhatan(1,c);% store them in a room
    end
 %% to tell have the new expanded state appeaered or not before
    for x=1:f
        for o=1:m
            if  closedstack{o}==tempstack{x} %check the closestack
                visited1=1;
                break
            else
                visited1=0;
            end
        end
        for y=1:openstack_index
            if openstack{y}==tempstack{x} % check the openstack
                visited2=1;
                break
            else
                visited2=0;
            end
        end
        if visited1==0&&visited2==0
            %if both don't have them before, store it
            openstack_index=openstack_index+1;
            openstack{openstack_index}=tempstack{x};
            openpath(openstack_index,:)=tempath(x,:);
        end
    end
    %%
    for l=1:f
        if goal==tempstack{l}
            steps=path(m,3);
            score=1;
            routemp=m;
            for r=1:steps
                route_index(r,:)=path(routemp,4);
                routemp=path(routemp,4);
                route{r}=stack{route_index(r,:)};
            end
            time=toc;
            totalnudes=o+openstack_index-2;
            return
        else score=0;
        end
    end
  %% move the state to the stack and expand
  % pick the state accoding to the Manhatan sistance
    index=find(openpath(1:openstack_index,5)==min(openpath(1:openstack_index,5)));
    stack{m+1}=openstack{index(1,1)};
    path(m+1,:)=openpath(index(1,1),:);
    closedstack{m+1}=openstack{index(1,1)};
    % in order to make the order right, i move the stored one
    openstack{index(1,1)}=[];
    openpath(index(1,1),:)=[];
    for z=index(1,1):openstack_index-1% reorderthe openstacks
        openstack{z}=openstack{z+1};
    end
    openstack_index=openstack_index-1;
   end