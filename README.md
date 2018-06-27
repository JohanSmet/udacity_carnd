# Udacity Self-Driving Car Engineer NanoDegree Projects

## Introduction

This repository contains the projects I submitted while taking the Self-Driving Car Engineer NanoDegree online course at [Udacity](http://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

In here you'll find a mix of projects pertaining to the field of self-driving cars. Topics include computer vision, deep learning, control, planning and decision making. The projects are implemented in either Python or C++.

## Repository Layout

All projects started out in a seperate GitHub repositories but after completing the first two terms I decided to merge them into one big repository to remove the clutter from my GitHub repository overview. Each project has its own branch and they are all merged into the master branch to get a nice overview.

I'm not a git-expert, so I used a simple procedure to merge the repositories I found on [StackOverflow](https://stackoverflow.com/questions/1425892/how-do-you-merge-two-git-repositories). To summarize:
```bash
# add a remote for the project
git remote add t1p1 <url to repository>
git fetch t1p1
# merge the project in it's own branch
git checkout -b t1p1
git merge --allow-unrelated-histories t1p1/master
# move the files to a subdirectory
mkdir term_1/01_lane_detection
git mv <files> term_1/01_lane_detection
# merge into master
git checkout master
git merge t1p1
# cleanup
git remote remove t1p1
```
