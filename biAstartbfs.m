clear all;
% MapInit=[0 0 0 0;0 0 0 0;0 0 0 0;1 2 3 4];
% MapInit=[0 0 2 3;0 0 0 0;0 0 0 0;4 0 0 1];
% MapInit=[1 0 0 0;0 0 0 0;4 0 0 2;0 0 0 3];
MapInit=[0 0 0 0;0 1 0 0;0 0 2 0;0 4 3 0];
goal=[0 0 0 0;0 1 0 0;0 2 0 0;0 3 0 4];
Map=MapInit;
Maptemp=Map;
biMapInit=goal;
bigoal=MapInit;
biMap=goal;
biMaptemp=biMap;
depth=zeros(100,1);
breth=zeros(100,1);
stack=cell(1000,1);
stack{1}=MapInit;
bistack=cell(1000,1);
bistack{1}=biMapInit;
tempstack=cell(4,1);
bitempstack=cell(4,1);
openstack=cell(1000,1);
openstack{1}=MapInit;
biopenstack=cell(1000,1);
biopenstack{1}=biMapInit;
closedstack=cell(1000,1);
closedstack{1}=MapInit;
biclosedstack=cell(1000,1);
biclosedstack{1}=biMapInit;
openstack_index=1;
biopenstack_index=1;
openpath=zeros(1000,5);
biopenpath=zeros(1000,5);
score=0;
path=zeros(1000,5);
bipath=zeros(1000,5);
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
    goal=bistack{m};
    [row,col]=find(goal==1);
    bisigna(1,:)=[row,col];
    [row,col]=find(goal==2);
    bisignb(1,:)=[row,col];
    [row,col]=find(goal==3);
    bisignc(1,:)=[row,col];
    [row,col]=find(goal==4);
    bisignblock(1,:)=[row,col];
    for c=1:stop-start
        [row,col]=find(tempstack{c}==1);
        signa(c,:)=[row,col];
        [row,col]=find(tempstack{c}==2);
        signb(c,:)=[row,col];
        [row,col]=find(tempstack{c}==3);
        signc(c,:)=[row,col];
        [row,col]=find(tempstack{c}==4);
        signblock(c,:)=[row,col];
        Manhatan(1,c)=tempath(c,3)+abs(signa(c,1)-bisigna(1,1))+abs(signa(c,2)-bisigna(1,2))+abs(signb(c,1)-bisignb(1,1))+abs(signb(c,2)-bisignb(1,2))+abs(signc(c,1)-bisignc(1,1))+abs(signc(c,2)-bisignc(1,2))+abs(signblock(c,1)-bisignblock(1,1))+abs(signblock(c,2)-bisignblock(1,2));
        tempath(c,5)=Manhatan(1,c);
    end
    for x=1:f
        for o=1:m
            if  closedstack{o}==tempstack{x}
                visited1=1;
                break
            else
                visited1=0;
            end
        end
        for y=1:openstack_index
            if openstack{y}==tempstack{x}
                visited2=1;
                break
            else
                visited2=0;
            end
        end
        if visited1==0&&visited2==0
            openstack_index=openstack_index+1;
            openstack{openstack_index}=tempstack{x};
            openpath(openstack_index,:)=tempath(x,:);
        end
    end
    
    %% Bidirection search
    goal=bistack{m};
    index=find(openpath(1:openstack_index,5)==min(openpath(1:openstack_index,5)));
    stack{m+1}=openstack{index(1,1)};
    path(m+1,:)=openpath(index(1,1),:);
    closedstack{m+1}=openstack{index(1,1)};
    openstack{index(1,1)}=[];
    openpath(index(1,1),:)=[];
    for z=index(1,1):openstack_index-1
        openstack{z}=openstack{z+1};
    end
    openstack_index=openstack_index-1;
    
    biMap=bistack{m};
    biMaptemp=biMap;
    direction_index=randperm(4);
    bitempath=zeros(4,4);
    bitempstack=cell(4,1);
    biManhatan=[];
    bif=0;
    for l=1:4
        b(l,:)=direction(direction_index(:,l),:);
    end
    [row,col]=find(biMap==4);
    bip=[row,col];
    temp=bip;
    start=bif;
    for n=1:4
        bip=bip+b(n,:);
        moverror=find(bip<1|bip>4);
        if (~isempty(moverror))
            bip=temp;
        elseif b(n,:)==bipath(m,1:2)
            bip=temp;
        else
            biMaptemp(bip(1,1),bip(1,2))=biMap(temp(1,1),temp(1,2));
            biMaptemp(temp(1,1),temp(1,2))=biMap(bip(1,1),bip(1,2));
            bif=bif+1;
            bitempstack{bif}=biMaptemp;
            bitempath(bif,1:2)=-b(n,:);
            bitempath(bif,3)=bipath(m,3)+1;
            bitempath(bif,4)=m;
            bip=temp;
            biMaptemp=biMap;
        end
    end
    stop=bif;
    bigoal=stack{m+1};
    [row,col]=find(bigoal==1);
    bisigna(1,:)=[row,col];
    [row,col]=find(bigoal==2);
    bisignb(1,:)=[row,col];
    [row,col]=find(bigoal==3);
    bisignc(1,:)=[row,col];
    [row,col]=find(bigoal==4);
    bisignblock(1,:)=[row,col];
    for c=1:stop-start
        [row,col]=find(bitempstack{c}==1);
        signa(c,:)=[row,col];
        [row,col]=find(bitempstack{c}==2);
        signb(c,:)=[row,col];
        [row,col]=find(bitempstack{c}==3);
        signc(c,:)=[row,col];
        [row,col]=find(bitempstack{c}==4);
        signblock(c,:)=[row,col];
        biManhatan(1,c)=bitempath(c,3)+abs(signa(c,1)-bisigna(1,1))+abs(signa(c,2)-bisigna(1,2))+abs(signb(c,1)-bisignb(1,1))+abs(signb(c,2)-bisignb(1,2))+abs(signc(c,1)-bisignc(1,1))+abs(signc(c,2)-bisignc(1,2))+abs(signblock(c,1)-bisignblock(1,1))+abs(signblock(c,2)-bisignblock(1,2));
        bitempath(c,5)=biManhatan(1,c);
    end
    for x=1:bif
        for o=1:m
            if  biclosedstack{o}==bitempstack{x}
                visited1=1;
                break
            else
                visited1=0;
            end
        end
        for y=1:biopenstack_index
            if biopenstack{y}==bitempstack{x}
                visited2=1;
                break
            else
                visited2=0;
            end
        end
        if visited1==0&&visited2==0
            biopenstack_index=biopenstack_index+1;
            biopenstack{biopenstack_index}=bitempstack{x};
            biopenpath(biopenstack_index,:)=bitempath(x,:);
        end
    end
    goal=stack{m+1};
    biindex=find(biopenpath(1:biopenstack_index,5)==min(biopenpath(1:biopenstack_index,5)));
    bistack{m+1}=biopenstack{biindex(1,1)};
    bipath(m+1,:)=biopenpath(biindex(1,1),:);
    biclosedstack{m+1}=biopenstack{biindex(1,1)};
    biopenstack{biindex(1,1)}=[];
    biopenpath(biindex(1,1),:)=[];
    for z=biindex(1,1):biopenstack_index-1
        biopenstack{z}=biopenstack{z+1};
    end
    biopenstack_index=biopenstack_index-1;
    %% check the states in stack,bistack,openstatck,biopenstack,
    %closestack and biclosestack, the reason for that because there always
    %one been moved.
    for q=1:biopenstack_index-1
        for w=1:openstack_index-1
            if (openstack{w}==biopenstack{q})
                score=1;
                steps=openpath(w,3);
                bisteps=biopenpath(q,3);
                %track the route, the path been stored in route and biroute
                routemp=w;
                biroutemp=q;
                route{1}=openstack{w};
                biroute{1}=biopenstack{q};
                route_index(1,:)=openpath(routemp,4);
                biroute_index(1,:)=biopenpath(biroutemp,4);
                for r=2:steps
                    route{r}=stack{route_index(r-1,:)};
                    route_index(r,:)=path(route_index(r-1,:),4);
                end
                for r=2:bisteps
                    biroute{r}=bistack{biroute_index(r-1,:)};
                    biroute_index(r,:)=bipath(biroute_index(r-1,:),4);
                end
                totalnudes=2*(m+1)+biopenstack_index+openstack_index;
                return
            elseif (openstack{w}==bistack{m+1})
                score=1;
                steps=openpath(w,3);
                bisteps=biopenpath(q,3);
                routemp=w;
                biroutemp=q;
                route{1}=openstack{w};
                biroute{1}=biopenstack{q};
                route_index(1,:)=openpath(routemp,4);
                biroute_index(1,:)=biopenpath(biroutemp,4);
                for r=2:steps
                    route{r}=stack{route_index(r-1,:)};
                    route_index(r,:)=path(route_index(r-1,:),4);
                end
                for r=2:bisteps
                    biroute{r}=bistack{biroute_index(r-1,:)};
                    biroute_index(r,:)=bipath(biroute_index(r-1,:),4);
                end
                totalnudes=2*(m+1)+biopenstack_index+openstack_index;
                return
            elseif (stack{m+1}==biopenstack{q})
                score=1;
                steps=openpath(w,3);
                bisteps=biopenpath(q,3);
                routemp=w;
                biroutemp=q;
                route{1}=openstack{w};
                biroute{1}=biopenstack{q};
                route_index(1,:)=openpath(routemp,4);
                biroute_index(1,:)=biopenpath(biroutemp,4);
                for r=2:steps
                    route{r}=stack{route_index(r-1,:)};
                    route_index(r,:)=path(route_index(r-1,:),4);
                end
                for r=2:bisteps
                    biroute{r}=bistack{biroute_index(r-1,:)};
                    biroute_index(r,:)=bipath(biroute_index(r-1,:),4);
                end
                totalnudes=2*(m+1)+biopenstack_index+openstack_index;
                return
            elseif (stack{m+1}==bistack{m+1})
                score=1;
                steps=openpath(w,3);
                bisteps=biopenpath(q,3);
                routemp=w;
                biroutemp=q;
                route{1}=openstack{w};
                biroute{1}=biopenstack{q};
                route_index(1,:)=openpath(routemp,4);
                biroute_index(1,:)=biopenpath(biroutemp,4);
                for r=2:steps
                    route{r}=stack{route_index(r-1,:)};
                    route_index(r,:)=path(route_index(r-1,:),4);
                end
                for r=2:bisteps
                    biroute{r}=bistack{biroute_index(r-1,:)};
                    biroute_index(r,:)=bipath(biroute_index(r-1,:),4);
                end
                totalnudes=2*(m+1)+biopenstack_index+openstack_index;
                return
            end
        end
    end
end