clear all;
MapInit=[0 0 0 0;0 0 0 0;0 0 0 0;1 2 3 4];
% MapInit=[0 0 2 3;0 0 0 0;0 0 0 0;4 0 0 1];
% MapInit=[1 0 0 0;0 0 0 0;4 0 0 2;0 0 0 3];
% MapInit=[0 0 0 0;0 1 0 0;0 0 2 0;0 4 3 0];
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
while score==0;
    m=m+1;
    Map=stack{m};
    Maptemp=Map;
    direction_index=randperm(4);
    for l=1:4
        b(l,:)=direction(direction_index(:,l),:);
    end
    [row,col]=find(Map==4);
    bip=[row,col];
    temp=bip;
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
            for x=1:j
                if stack{x}==Maptemp
                    bip=temp;
                    Maptemp=Map;
                else
                    j=j+1;
                    stack{j}=Maptemp;
                    path(j,1:2)=-b(n,:);
                    path(j,3)=path(m,3)+1;
                    path(j,4)=m;
                    bip=temp;
                    Maptemp=Map;
                    break %only expand to one state and store it
                end
            end
            break
        end
    end
    if goal==stack{m+1};
        steps=path(m,3);
        score=1;
        routemp=m+1;
        for r=1:steps
            route_index(r,:)=path(routemp,4);
            routemp=path(routemp,4);
            route{r}=stack{route_index(r,:)};
        end
        return
    else score=0;
    end
end