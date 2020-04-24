# cars

## Arduino:
suivi_piste.ino : code source arduino principal permettant à la voiture de suivre une piste basique sans intersection. Téléverser simplement sur la Teensy4 et mettre sous tension la voiture, l’ESC se calibre automatiquement et la voiture démarre.
Ce fichier possède du code pour détection de passage piéton. II a été mis en commentaire car n’étant pas complètement opérationnel

algo_arret.ino : code source permettant à la voiture de s’arrêter face à un obstacle

calibrage.ino : code source servant à calibrer l’ESC de la voiture 1/10. Il n’est plus utile avec la 1/18 mais garde son importance en tant qu’archive

TFmini_loop.ino : code source du fonctionnement du Lidar TFMini

## Unity :
Pour lancer la simulation il faut télécharger un package et l’importer dans un projet Unity. Si PackageCache  est présent dans le répertoire Library,  il important de le supprimer. Ensuite dans le dossier Assets/Scenes il faut faire un double clic sur NXP Cup track. On entre alors dans le mode play.

Package Suivi Circuit : ce package est celui de base fournit par le client, seul le script AutoDrive.cs a été modifié pour essayer de suivre le circuit

Package obstacle : ce package comprend l’ajout d’un cube 3D et un algorithme de détection et d’évitement d’obstacle dans le fichier AutoDrive.cs

train.py : Script pour trainer un réseau neuronal convolutif en utilisant la bibliothèque PyTorch. Le jeu de données doit se trouver dans un dossier appelé "ds"


## Branches

```master``` est la branche de rendu, où le code est fonctionnel
