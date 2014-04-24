#import "AppDelegate.h"
#import <ScriptingBridge/ScriptingBridge.h>
#import <objc/runtime.h>

NSString * const NotificationCenterUIBundleID = @"com.apple.notificationcenterui";


@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)notification;
{
    NSArray *runningProcesses = [[[NSWorkspace sharedWorkspace] runningApplications] valueForKey:@"bundleIdentifier"];
    if ([runningProcesses indexOfObject:NotificationCenterUIBundleID] == NSNotFound) {
      NSLog(@"[!] Unable to post a notification for the current user (%@), as it has no running NotificationCenter instance.", NSUserName());
      exit(1);
    }

    NSArray *arguments = [[NSProcessInfo processInfo] arguments];
    
    NSString *title = @"No title";
    if ([arguments count] > 1)
    {
        title = arguments[1];
    }
    
    NSString *message = @"No message";
    if ([arguments count] > 2)
    {
        message = arguments[2];
    }
    
    NSUserNotification *userNotification = [NSUserNotification new];
    userNotification.title = title;
//    userNotification.subtitle = message;
    userNotification.informativeText = message;
    
    NSUserNotificationCenter *center = [NSUserNotificationCenter defaultUserNotificationCenter];
    center.delegate = self;
    [center scheduleNotification:userNotification];
}

- (BOOL)userNotificationCenter:(NSUserNotificationCenter *)center
     shouldPresentNotification:(NSUserNotification *)userNotification;
{
  return YES;
}

// Once the notification is delivered we can exit.
- (void)userNotificationCenter:(NSUserNotificationCenter *)center
        didDeliverNotification:(NSUserNotification *)userNotification;
{
  exit(0);
}

@end
