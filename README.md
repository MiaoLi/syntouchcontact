# syntouchcontact
This package publish the contact points and the contact forces using BioTac

Dependence: [syntouchpublisher](https://github.com/MiaoLi/syntouchpublisher)

To run: roslaunch syntouchcontact syntouchcontact.launch

## Contact points estimation
### Coordinates
<img src="/misc/forcecalibration.png" alt="alt text" width="150" height="200">

### approach
* We use the weighted average to estimate the contact points and the accuracy is around 3mm

## Contact forces estimation
* We use Gaussian Mixture Model (GMM) to estimate the contact forces (3-dimensional) and the normal contact force accuracy is 
around 0.1N and the tangential force accuracy is around 0.3N.