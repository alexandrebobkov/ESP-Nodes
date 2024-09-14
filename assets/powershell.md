# Get-Help

# Get-Children

Gets items in a specified location. To list the folders in my drive C, I will run
the command below:

`Get-ChildItem c:/`

This will list all the top-level folders. To list all files, folders include subfolders use the -Recurse parameter.

# Copy-Item and Move-Item

You could use the Get-ChildItem Cmdlet to list items in a folder, then pipe
the result to Copy-Item Cmdlet to copy the items to a new location. The
command below will do the job:

`Get-ChildItem C:\Dropbox | Copy-Item -Destination C:\NewFolder`

The above PowerShell command will only copy the top-level folders and
files - it will NOT copy sub-folders and files. To copy all files and folders
including sub-folders, include the -Recurse parameter in the Get-ChildItem
command as shown below:

`Get-ChildItem C:\Dropbox -Recurse | Copy-Item -Destination C:\NewFolder`

While the Copy-Item Cmdlet copies items from one location to another the
Move-Item Cmdlet moves the item.


