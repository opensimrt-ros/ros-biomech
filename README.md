# ros\_biomech mother-repo


This should have all the catkin\_ws packages you are using.


## Caveats:

Currently, I am working alone on changing this on devel and pushing to master as I get something new working. This is not a suitable long term strategy for a lot of people working on this. 

I think it would be best to make this repo read only for most people and make it forkable, so that you can work on your own versions of things and just submit pull requests. 

So ***PLEASE*** don't wreck it. Now everyone I added has write permissions and it is possible to create a lot of damage. 

### Submodules tracks commited content, not pushed content

One thing I already destroyed is making a change to a subpackage we didn't fork and setting the head to include a commit that wasn't pushed. Git and Github will allow this, but when you try to pull it, it won't find that commit and it will fail!

Git status shows commited content as ok, but it won't tell you if you forgot to push it upstream (this was one of the reasons why I lost data the last time).

### Git will only push the current branch

Make sure to microcommit *and push*, and then later squash feature / bugfix branches. I have been lazy with this, but it is important, because the latest version of git defaults to not pushing all the branches you have commits, only the current branch. This can cause loss of data if you are working on multiple things without pushing (you may think, it's not done, it isn't working, why should I push), this is the reason for working with feature / bugfix branches anyway, so this is a better way of keeping track of code. 

### Git will not show a star on branches that have untracked files

Maybe this is a fault of my PS1 bar implementation, but it doesn't complain about untracked files. If you just go to a directory and don't type git status, you won't know if eveyrthing is committed either, so make sure you adjust to that. 

