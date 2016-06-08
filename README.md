# syntouchcontact
This package publish the contact points and the contact forces using BioTac


## Contact points estimation
### Coordinates
y^

 |

 |

 |
 
z@-------->x

### approach
* We use the weighted average to estimate the contact points and the accuracy is around 3mm

## Contact forces estimation
* We use Gaussian Mixture Model (GMM) to estimate the contact forces (3-dimensional) and the normal contact force accuracy is 
around 0.1N and the tangential force accuracy is around 0.3N.