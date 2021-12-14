open('THREE_PID.slx')
load('ELBOW_prop..mat')

%% Problem Definition
objfun = @(K)control_ga(K);

global K_p K_i

N = 2;

K_min = [0 0 0 0 0 0];
K_max = [30 15 5 5 15 10];

%% GA Parameter Initialization

popsize = 200;
MaxIt = 100;

nb = [20 20 20 20 20 20];
Nt = sum(nb);

selection_rate = 0.25;
mutation_rate = 0.1;

total_mutations = floor(mutation_rate*Nt*popsize);

%% Population Initialization

initialpop = round(rand(posize, Nt));

a = 0;
b = 0;

for i = 1:N
    a = b+1;
    b = b+nb(i);

    DV = bi2de(initialpop(:, (a:b)));
    K(:,i) = K_min(i) + (((K_max(i) - K_min(i))*DV)/(2^nb(i) - 1));
end

%% Fitness Evaluation

for i = 1:popsize
    K_p = K(i,1);
    K_i = K(i,2);

    sim('THREE_PID')
    ch1 = ans.ISE1(end);
    ch2 = ans.ISE1(end);
    ch3 = ans.ISE1(end);
    
    ch = [ch1,ch2,ch3];
    fit(i,3) = (ch);
    fitness(i,3) = 1./(1+fit(i,3));
end

%% Selection of mating pool
for ii = 1:MaxIt
    fitprob = fitness./sum(fitness);
    cumprob = cumsum(fitprob);

    for i = 1:popsize
        r = rand(1,3);
        for j = 1:popsize
            if r < cumprob(j,1:3)
                newpop(i,:) = initialpop(j,:);
                break
            end
        end
    end

    %% Parent Selection
    l = 0;
    m = 0;

    Parent = [];
    rempop = [];

    for k = 1:popsize
        R = rand;
        if R < selection_rate
            l = l + 1;
            Parent(l,:) = newpop(k,:);
        else
            m = m+1;
            rempop(m,:) = newpop(k,:);
        end
    end

    %% Crossover
    offspring = [];

    if size(Parent,1) > 1
        for i = (size(Parent,1)-1)
            r = randi(Nt);
            offspring(i,:) = [Parent(i,1:r) Parent(i+1, r+1:Nt)];
        end
        r = randi(Nt);
        offspring(i+1, :) = [Parent(i+1,1:r) Parent(1,r+1:Nt)];
        newpop = [offspring;rempop];
    end

    %% Mutations
    totgen = popsize*Nt;

    for i = 1:total_mutations
        r = randi(totgen);

        row = (ceil(r/Nt));

        if rem(r,Nt) == 0
            col = Nt;
        else
            col = rem(r,Nt);
        end

        if newpop(row,col) == 1
            newpop(row,col) = 0;
        else
            newpop(row,col) = 1;
        end
    end

    %% Fitness Evaluation of new population
    a = 0;
    b = 0;

    for i = 1:N
        a = b+1;
        b = b+nb(i);
        DV = bi2de(newpop(:, (a:b)));
        K(:,i) = K_min(i) + ((K_max(i) - K_min(i))*DV) / (2^nb(i) - 1);
    end

    for i = 1:popsize
        K_p = K(i,1);
        K_i = K(i,2);
        sim('THREE_PID')
        ch1 = ans.ISE1(end);
        ch2 = ans.ISE1(end);
        ch3 = ans.ISE1(end);
        ch = [ch1 ch2 ch3];
        fit(i,3) = (ch);
        fitness(i,3) = 1./(1+fit(i,3));
    end

    initialpop = newpop;
end