clear();
fid = fopen('log_module_position.txt'); %ouverture du fichier de données
% log_module_position.txt
%log_drone_pos_and_velocity.txt
firstLine = fgets(fid);     %permet de retrouver le titre des colonnes par la suite
firstLine = textscan(firstLine,'%s');
L = size(firstLine{1});     %pour connaitre le nombre de colonnes
L = L(1);
formatDonnees = "";
for i = 1:L
    formatDonnees=formatDonnees+"%s";
end


%%recherche de la mise en page des données
colonnesDeTitre = 1;  %s'il y a le titre de chaque colonne réécrit à chaque ligne
for i=1:L
    if size(str2num(char(firstLine{1}{i})))==1
        colonnesDeTitre=2; %chaque colonne n'est que valeurs.
    end
end


titres = strings(L/colonnesDeTitre,1);    %le nom de chaque colonnes
for i = 1:colonnesDeTitre:L
    titres(i) = firstLine{1}{i};
end
% pour la lisibilité : 
titres = strrep(titres,"_","\_");
titres = strrep(titres,"°","ation");
if size(char(titres(1)))==[1 1] titres(1) = "temps"; end
if size(char(titres(2)))==[1 1] titres(2) = "position X"; end
if size(char(titres(3)))==[1 1] titres(3) = "position Y"; end
%%Ici on récupère finalement les données qui nous intéressent
C = textscan(fid, formatDonnees,'Headerlines',2-colonnesDeTitre);   %renvoie le tableau de toutes les données
fclose(fid);
N = size(C{end}); % le nombre de lignes dans le tableau
N = N(1);
results = zeros(L,N); %passage sous forme de matrice
%et mise en forme
for i = colonnesDeTitre:colonnesDeTitre:L
    for j=1:N
        a = C{i}{j};
        a = str2double(a);
        results(i,j) = a(1);
    end
end

%ici, le but est de trouvé comment afficher les résultats et donc de
%trouver 2 nombres qui décomposes au mieux le nombre de résultats.
%ce sont les 2 nombres qu'il restera dans plotSize
if isprime(L/colonnesDeTitre) && L/colonnesDeTitre > 3
    plotSize = factor(L/colonnesDeTitre+1);
else
    plotSize = factor(L/colonnesDeTitre);
end
if L/colonnesDeTitre <=3
    plotSize = L/colonnesDeTitre;
end
nbfactor =size(plotSize);
while nbfactor(2)>=3
    plotSize = [plotSize(1)*plotSize(2) plotSize(3:end)];
    nbfactor =size(plotSize);
end
plotSize = sort(plotSize);
if size(plotSize)==[1 1]
    plotSize = [1 plotSize];
end



%%Affichage des résultats
figure
for i = 2*colonnesDeTitre:colonnesDeTitre:L
    subplot(plotSize(1),plotSize(2),i/colonnesDeTitre);   %,'replace')
    %titre = char(titres(i/colonnesDeTitre)+" en fonction de "+titres(1));
    plot(results(colonnesDeTitre,:),results(i,:));
    ylabel(titres(i/colonnesDeTitre));
    xlabel(titres(1));
end
%La courbe de Y en fonction de X du déplacement du robot :
subplot(plotSize(1),plotSize(2),1)
plot(results(2*colonnesDeTitre,:),results(3*colonnesDeTitre,:));
xlabel(titres(2));
ylabel(titres(3));
%titre('trajectoire du robot et ses données en fonction du temps');

%d'autres plots séparés pour mieux voir
figure
subplot(1,3,1)
plot(results(2*colonnesDeTitre,:),results(3*colonnesDeTitre,:));
xlabel(titres(2));
ylabel(titres(3));
for i = 2*colonnesDeTitre:colonnesDeTitre:3*colonnesDeTitre
    subplot(1,3,i/colonnesDeTitre);   %,'replace')
    plot(results(colonnesDeTitre,:),results(i,:));
    ylabel(titres(i/colonnesDeTitre));
    xlabel(titres(1));
end
